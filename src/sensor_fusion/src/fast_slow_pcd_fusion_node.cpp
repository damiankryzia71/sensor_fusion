#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointXYZ = pcl::PointXYZ;

class FastSlowPointCloudFusionNode : public rclcpp::Node
{
public:
    FastSlowPointCloudFusionNode()
        : Node("fast_slow_pointcloud_fusion_node")
    {
        this->declare_parameter<std::string>("fast_topic");
        this->declare_parameter<std::string>("slow_topic");
        this->declare_parameter<std::string>("fused_topic");
        this->declare_parameter<std::string>("transform_yaml_path");
        this->declare_parameter<std::string>("output_frame_id");

        this->declare_parameter<std::vector<double>>("self_filter.min", {-0.5, -0.5, -0.1});
        this->declare_parameter<std::vector<double>>("self_filter.max", {0.5, 0.5, 0.5});
        this->declare_parameter<std::vector<double>>("self_filter.offset", {0.0, 0.0, 0.0});

        std::string fast_topic = this->get_parameter("fast_topic").as_string();
        std::string slow_topic = this->get_parameter("slow_topic").as_string();
        std::string fused_topic = this->get_parameter("fused_topic").as_string();
        std::string yaml_path = this->get_parameter("transform_yaml_path").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        
        self_filter_min_ = this->get_parameter("self_filter.min").as_double_array();
        self_filter_max_ = this->get_parameter("self_filter.max").as_double_array();
        self_filter_offset_ = this->get_parameter("self_filter.offset").as_double_array();

        try {
            if (yaml_path.empty()) {
                throw std::runtime_error("Parameter 'transform_yaml_path' is not set or is empty.");
            }
            slow_to_fast_transform_ = parseTrMatrixFromYAML(yaml_path);
            RCLCPP_INFO(this->get_logger(), "Successfully loaded transformation matrix from %s", yaml_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load transform matrix: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        
        slow_cloud_ = std::make_shared<pcl::PointCloud<PointXYZ>>();
        slow_cloud_transformed_ = std::make_shared<pcl::PointCloud<PointXYZ>>();
        fast_cloud_ = std::make_shared<pcl::PointCloud<PointXYZ>>();
        fused_cloud_ = std::make_shared<pcl::PointCloud<PointXYZ>>();
        filtered_cloud_ = std::make_shared<pcl::PointCloud<PointXYZ>>();

        initializeSelfFilter();

        rclcpp::SensorDataQoS qos_profile;

        pub_fused_ = this->create_publisher<PointCloud2>(fused_topic, qos_profile);

        sub_slow_ = this->create_subscription<PointCloud2>(
            slow_topic, qos_profile,
            std::bind(&FastSlowPointCloudFusionNode::callback_slow, this, std::placeholders::_1));

        sub_fast_ = this->create_subscription<PointCloud2>(
            fast_topic, qos_profile,
            std::bind(&FastSlowPointCloudFusionNode::callback_fast, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Subscribing to '%s' (fast) and '%s' (slow). Publishing to '%s'.",
            fast_topic.c_str(), slow_topic.c_str(), fused_topic.c_str());
    }

private:
    Eigen::Matrix4f parseTrMatrixFromYAML(const std::string& filepath) {
        YAML::Node config = YAML::LoadFile(filepath);
        Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
        for (int r = 1; r <= 4; ++r) {
            for (int c = 1; c <= 4; ++c) {
                std::string key = "Tr" + std::to_string(r) + std::to_string(c);
                if (!config[key]) {
                    throw std::runtime_error("Missing YAML key: " + key);
                }
                tr(r - 1, c - 1) = config[key].as<float>();
            }
        }
        return tr;
    }

    void initializeSelfFilter() {
        if (self_filter_min_.size() != 3 || self_filter_max_.size() != 3 || self_filter_offset_.size() != 3) {
            RCLCPP_WARN(get_logger(), "self_filter parameters must have 3 elements each; self-filter will be disabled.");
            return;
        }

        Eigen::Vector4f min_pt(
            static_cast<float>(self_filter_min_[0] + self_filter_offset_[0]),
            static_cast<float>(self_filter_min_[1] + self_filter_offset_[1]),
            static_cast<float>(self_filter_min_[2] + self_filter_offset_[2]),
            1.0f);
        Eigen::Vector4f max_pt(
            static_cast<float>(self_filter_max_[0] + self_filter_offset_[0]),
            static_cast<float>(self_filter_max_[1] + self_filter_offset_[1]),
            static_cast<float>(self_filter_max_[2] + self_filter_offset_[2]),
            1.0f);

        self_crop_filter_.setMin(min_pt);
        self_crop_filter_.setMax(max_pt);
        self_crop_filter_.setNegative(true);
        self_filter_initialized_ = true;
    }

    void applySelfFilter(const pcl::PointCloud<PointXYZ>::ConstPtr& in, pcl::PointCloud<PointXYZ>& out) {
        if (!self_filter_initialized_) {
            out = *in;
            return;
        }
        self_crop_filter_.setInputCloud(in);
        self_crop_filter_.filter(out);
    }

    void callback_slow(const PointCloud2::SharedPtr msg)
    {
        slow_cloud_->clear();
        slow_cloud_transformed_->clear();

        pcl::fromROSMsg(*msg, *slow_cloud_);

        pcl::transformPointCloud(*slow_cloud_, *slow_cloud_transformed_, slow_to_fast_transform_);

        std::lock_guard<std::mutex> lock(slow_cloud_mutex_);
        latest_slow_cloud_transformed_ = slow_cloud_transformed_;
        is_slow_cloud_fresh_ = true; // Mark the new slow cloud as fresh
    }

    void callback_fast(const PointCloud2::SharedPtr fast_msg)
    {
        bool use_slow_cloud = false;
        pcl::PointCloud<PointXYZ>::Ptr slow_cloud_transformed_copy;

        {
            std::lock_guard<std::mutex> lock(slow_cloud_mutex_);
            if (!latest_slow_cloud_transformed_) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for the first message from the slow topic...");
                return;
            }
            
            if (is_slow_cloud_fresh_) {
                slow_cloud_transformed_copy = latest_slow_cloud_transformed_;
                use_slow_cloud = true;
                is_slow_cloud_fresh_ = false; // Mark as used
            }
        }
        
        fast_cloud_->clear();
        fused_cloud_->clear();
        filtered_cloud_->clear();

        pcl::fromROSMsg(*fast_msg, *fast_cloud_);

        *fused_cloud_ = *fast_cloud_;

        if (use_slow_cloud) {
            *fused_cloud_ += *slow_cloud_transformed_copy;
        }
        
        applySelfFilter(fused_cloud_, *filtered_cloud_);

        auto fused_msg = std::make_unique<PointCloud2>();
        pcl::toROSMsg(*filtered_cloud_, *fused_msg);

        fused_msg->header = fast_msg->header;
        fused_msg->header.frame_id = output_frame_id_;

        pub_fused_->publish(std::move(fused_msg));
    }

    rclcpp::Publisher<PointCloud2>::SharedPtr pub_fused_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_fast_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_slow_;

    pcl::PointCloud<PointXYZ>::Ptr latest_slow_cloud_transformed_;
    pcl::PointCloud<PointXYZ>::Ptr slow_cloud_;
    pcl::PointCloud<PointXYZ>::Ptr slow_cloud_transformed_;
    pcl::PointCloud<PointXYZ>::Ptr fast_cloud_;
    pcl::PointCloud<PointXYZ>::Ptr fused_cloud_;
    pcl::PointCloud<PointXYZ>::Ptr filtered_cloud_;

    Eigen::Matrix4f slow_to_fast_transform_;
    std::mutex slow_cloud_mutex_;
    std::vector<double> self_filter_min_, self_filter_max_, self_filter_offset_;
    std::string output_frame_id_;
    
    pcl::CropBox<PointXYZ> self_crop_filter_;
    bool self_filter_initialized_ = false;
    bool is_slow_cloud_fresh_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastSlowPointCloudFusionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
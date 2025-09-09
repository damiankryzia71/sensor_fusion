#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

class PointCloudFastSlowFusionNode : public rclcpp::Node
{
    public:
        PointCloudFastSlowFusionNode() : Node("pcd_fast_slow_fusion_node")
        {
            declare_parameter<std::string>("fast_topic");
            declare_parameter<std::string>("slow_topic");
            declare_parameter<std::string>("output_topic");
            declare_parameter<std::string>("tr_matrix_config_path");
            declare_parameter<std::string>("output_frame_id");

            if (!get_parameter("fast_topic", fast_topic_))
            {
                RCLCPP_ERROR(get_logger(), "Required parameter fast_topic is missing. Aborting...");
                throw std::runtime_error("Runtime error");
            }
            if (!get_parameter("slow_topic", slow_topic_))
            {
                RCLCPP_ERROR(get_logger(), "Required parameter slow_topic is missing. Aborting...");
                throw std::runtime_error("Runtime error");
            }
            if (!get_parameter("output_topic", output_topic_))
            {
                RCLCPP_ERROR(get_logger(), "Required parameter output_topic is missing. Aborting...");
                throw std::runtime_error("Runtime error");
            }
            if (!get_parameter("tr_matrix_config_path", tr_matrix_config_path_))
            {
                RCLCPP_ERROR(get_logger(), "Required parameter tr_matrix_config_path is missing. Aborting...");
                throw std::runtime_error("Runtime error");
            }
            if (!get_parameter("output_frame_id", output_frame_id_))
            {
                RCLCPP_ERROR(get_logger(), "Required parameter output_frame_id is missing. Aborting...");
                throw std::runtime_error("Runtime error");
            }

            try 
            {
                tr_ = parse_tr_matrix_from_yaml(tr_matrix_config_path_);
            }
            catch (const YAML::ParserException& e) 
            {
                RCLCPP_ERROR(get_logger(), "An error occured when parsing YAML file: %s", e.what());
                throw std::runtime_error("Runtime error");
            }
            catch (const YAML::BadFile& e) 
            {
                RCLCPP_ERROR(get_logger(), "Could not open YAML file: %s", e.what());
                throw std::runtime_error("Runtime error");
            }
            catch(const std::exception& e) 
            {
                RCLCPP_ERROR(get_logger(), "An error occured when parsing YAML file: %s", e.what());
                throw std::runtime_error("Runtime error");
            }

            self_filter_min_ = declare_parameter<std::vector<double>>("self_filter_min", {-0.5, -0.5, -0.1});
            self_filter_max_ = declare_parameter<std::vector<double>>("self_filter_max", {0.5, 0.5, 0.5});
            self_filter_offset_ = declare_parameter<std::vector<double>>("self_filter_offset", {0.0, 0.0, 0.0});

            max_slow_age_ms_ = declare_parameter<double>("max_slow_age_ms", 300.0);

            sub_fast_ = create_subscription<sensor_msgs::msg::PointCloud2>(fast_topic_, rclcpp::SensorDataQoS(), 
                std::bind(&PointCloudFastSlowFusionNode::callback_fast, this, std::placeholders::_1));
            sub_slow_ = create_subscription<sensor_msgs::msg::PointCloud2>(slow_topic_, rclcpp::SensorDataQoS(), 
                std::bind(&PointCloudFastSlowFusionNode::callback_slow, this, std::placeholders::_1));

            pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

            RCLCPP_INFO(get_logger(), "Publishing fused PointCloud2 to topic: %s", output_topic_.c_str());
        }

    private:
        void callback_fast(const sensor_msgs::msg::PointCloud2& fast) 
        {
            sensor_msgs::msg::PointCloud2 slow;
            bool use_slow = false;
            {
                std::lock_guard<std::mutex> lock(slow_mutex_);
                if (new_slow_)
                {
                    const double age_ms = (rclcpp::Time(fast.header.stamp) - rclcpp::Time(latest_slow_transformed_.header.stamp)).seconds() * 1e3;
                    if (age_ms <= max_slow_age_ms_) 
                    {
                        slow = latest_slow_transformed_;
                        use_slow = true;
                    }
                }
            }

            sensor_msgs::msg::PointCloud2 out;
            if (use_slow) out = fuse_pcd(fast, slow);
            else out = fast;

            out.header.frame_id = output_frame_id_;
            apply_self_filter(out);
            pub_->publish(out);
        }

        void callback_slow(const sensor_msgs::msg::PointCloud2& slow) 
        {
            std::lock_guard<std::mutex> lock(slow_mutex_);
            pcl::PointCloud<pcl::PointXYZ> pb, pb_tr;
            pcl::fromROSMsg(slow, pb);
            pcl::transformPointCloud(pb, pb_tr, tr_);
            pcl::toROSMsg(pb_tr, latest_slow_transformed_);
            latest_slow_transformed_.header = slow.header;
            latest_slow_transformed_.header.frame_id = output_frame_id_;
            new_slow_ = true;
        }

        sensor_msgs::msg::PointCloud2 fuse_pcd(const sensor_msgs::msg::PointCloud2& a, const sensor_msgs::msg::PointCloud2& b) 
        {
            pcl::PointCloud<pcl::PointXYZ> pa, pb, pfused;
            pcl::fromROSMsg(a, pa);
            pcl::fromROSMsg(b, pb);
            pfused = pa;
            pfused += pb;
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(pfused, out);
            out.header.stamp = a.header.stamp;
            return out;
        }

        void apply_self_filter(sensor_msgs::msg::PointCloud2& msg) 
        {
            const auto header = msg.header;

            if (self_filter_min_.size() != 3 || self_filter_max_.size() != 3 || self_filter_offset_.size() != 3) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "self_filter_* parameters must have 3 elements each; skipping self-filter.");
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> in, out;
            pcl::fromROSMsg(msg, in);

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

            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(in.makeShared());
            crop.setMin(min_pt);
            crop.setMax(max_pt);
            crop.setNegative(true);
            crop.filter(out);

            pcl::toROSMsg(out, msg);
            msg.header = header;
        }

        Eigen::Matrix4f parse_tr_matrix_from_yaml(const std::string& filepath) 
        {
            YAML::Node config = YAML::LoadFile(filepath);
            Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
            for (int r = 1; r <= 3; ++r) {
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

        std::mutex slow_mutex_;
        bool new_slow_ = false;
        double max_slow_age_ms_;
        sensor_msgs::msg::PointCloud2 latest_slow_transformed_;
        std::string fast_topic_, slow_topic_, output_topic_, tr_matrix_config_path_, output_frame_id_;
        std::vector<double> self_filter_min_, self_filter_max_, self_filter_offset_;
        Eigen::Matrix4f tr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fast_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_slow_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFastSlowFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
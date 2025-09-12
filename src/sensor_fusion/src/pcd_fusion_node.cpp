#include <string>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

using sensor_msgs::msg::PointCloud2;
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;

class PointCloudFusionNode : public rclcpp::Node {
public:
  PointCloudFusionNode() : Node("pointcloud_fusion_node") {
    this->declare_parameter<std::string>("input_topic1");
    this->declare_parameter<std::string>("input_topic2");
    this->declare_parameter<std::string>("output_topic");
    this->declare_parameter<std::string>("tr_matrix_config_path");
    this->declare_parameter<std::string>("output_frame_id");

    self_filter_min_ = this->declare_parameter<std::vector<double>>("self_filter_min", {-0.5, -0.5, -0.1});
    self_filter_max_ = this->declare_parameter<std::vector<double>>("self_filter_max", {0.5, 0.5, 0.5});
    self_filter_offset_ = this->declare_parameter<std::vector<double>>("self_filter_offset", {0.0, 0.0, 0.0});

    input_topic1_ = this->get_parameter("input_topic1").as_string();
    input_topic2_ = this->get_parameter("input_topic2").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    tr_matrix_config_path_ = this->get_parameter("tr_matrix_config_path").as_string();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    
    cloud1_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud2_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    transformed_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    fused_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    initializeSelfFilter();

    try {
        tr_ = parseTrMatrixFromYAML(tr_matrix_config_path_);
    }
    catch (const YAML::ParserException& e) {
        RCLCPP_ERROR(get_logger(), "An error occured when parsing YAML file: %s", e.what());
        throw std::runtime_error("Runtime error");
    }
    catch (const YAML::BadFile& e) {
        RCLCPP_ERROR(get_logger(), "Could not open YAML file: %s", e.what());
        throw std::runtime_error("Runtime error");
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "An error occured when parsing YAML file: %s", e.what());
        throw std::runtime_error("Runtime error");
    }

    rclcpp::SubscriptionOptions sub_opts1;
    rclcpp::SubscriptionOptions sub_opts2;

    auto group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opts1.callback_group = group;
    sub_opts2.callback_group = group;

    rclcpp::SensorDataQoS qos_profile;

    sub1_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(this, input_topic1_, qos_profile.get_rmw_qos_profile(), sub_opts1);
    sub2_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(this, input_topic2_, qos_profile.get_rmw_qos_profile(), sub_opts2);

    sync_ = std::make_unique<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(50), *sub1_, *sub2_);
    sync_->registerCallback(std::bind(&PointCloudFusionNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    pub_ = create_publisher<PointCloud2>(output_topic_, qos_profile);

    RCLCPP_INFO(get_logger(), "Publishing fused PointCloud2 to topic: %s", output_topic_.c_str());
  }

private:
  void callback(const PointCloud2::ConstSharedPtr a, const PointCloud2::ConstSharedPtr b) {
    cloud1_->clear();
    cloud2_->clear();
    transformed_cloud_->clear();
    fused_cloud_->clear();
    filtered_cloud_->clear();

    pcl::fromROSMsg(*a, *cloud1_);
    pcl::fromROSMsg(*b, *cloud2_);
    pcl::transformPointCloud(*cloud2_, *transformed_cloud_, tr_);

    *fused_cloud_ = *cloud1_;
    *fused_cloud_ += *transformed_cloud_;

    applySelfFilter(fused_cloud_, *filtered_cloud_);

    PointCloud2 out;
    pcl::toROSMsg(*filtered_cloud_, out);
    out.header.frame_id = output_frame_id_;
    out.header.stamp = (rclcpp::Time(a->header.stamp) > rclcpp::Time(b->header.stamp)) ? a->header.stamp : b->header.stamp;
    pub_->publish(out);
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

        crop_box_.setMin(min_pt);
        crop_box_.setMax(max_pt);
        crop_box_.setNegative(true);
        self_filter_initialized_ = true;
  }


  void applySelfFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, pcl::PointCloud<pcl::PointXYZ>& out) {
      if (!self_filter_initialized_) {
          out = *in;
          return;
      }
      crop_box_.setInputCloud(in);
      crop_box_.filter(out);
  }

  Eigen::Matrix4f parseTrMatrixFromYAML(const std::string& filepath) {
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

  std::string input_topic1_, input_topic2_, output_topic_, tr_matrix_config_path_, output_frame_id_;
  std::vector<double> self_filter_min_, self_filter_max_, self_filter_offset_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, cloud2_, transformed_cloud_, fused_cloud_, filtered_cloud_;
  bool self_filter_initialized_ = false;
  pcl::CropBox<pcl::PointXYZ> crop_box_;
  Eigen::Matrix4f tr_;
  std::unique_ptr<message_filters::Subscriber<PointCloud2>> sub1_;
  std::unique_ptr<message_filters::Subscriber<PointCloud2>> sub2_;
  std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFusionNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
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
typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> ApproxPolicy;

class PointCloudFusionNode : public rclcpp::Node {
public:
  PointCloudFusionNode() : Node("pointcloud_fusion_node") {
    declare_parameter<std::string>("input_topic1");
    declare_parameter<std::string>("input_topic2");
    declare_parameter<std::string>("output_topic");
    declare_parameter<std::string>("tr_matrix_config_path");
    declare_parameter<std::string>("output_frame_id");

    if (!get_parameter("input_topic1", input_topic1_)) {
        RCLCPP_ERROR(get_logger(), "Required parameter input_topic1 is missing. Aborting...");
        throw std::runtime_error("Runtime error");
    }
    if (!get_parameter("input_topic2", input_topic2_)) {
        RCLCPP_ERROR(get_logger(), "Required parameter input_topic2 is missing. Aborting...");
        throw std::runtime_error("Runtime error");
    }
    if (!get_parameter("output_topic", output_topic_)) {
        RCLCPP_ERROR(get_logger(), "Required parameter output_topic is missing. Aborting...");
        throw std::runtime_error("Runtime error");
    }
    if (!get_parameter("tr_matrix_config_path", tr_matrix_config_path_)) {
        RCLCPP_ERROR(get_logger(), "Required parameter tr_matrix_config_path is missing. Aborting...");
        throw std::runtime_error("Runtime error");
    }
    if (!get_parameter("output_frame_id", output_frame_id_)) {
        RCLCPP_ERROR(get_logger(), "Required parameter output_frame_id is missing. Aborting...");
        throw std::runtime_error("Runtime error");
    }

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

    self_filter_min_ = declare_parameter<std::vector<double>>("self_filter_min", {-0.5, -0.5, -0.1});
    self_filter_max_ = declare_parameter<std::vector<double>>("self_filter_max", {0.5, 0.5, 0.5});
    self_filter_offset_ = declare_parameter<std::vector<double>>("self_filter_offset", {0.0, 0.0, 0.0});

    rclcpp::SubscriptionOptions sub_opts1;
    rclcpp::SubscriptionOptions sub_opts2;

    auto group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opts1.callback_group = group;
    sub_opts2.callback_group = group;

    sub1_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(this, input_topic1_, rclcpp::SensorDataQoS().get_rmw_qos_profile(), sub_opts1);
    sub2_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(this, input_topic2_, rclcpp::SensorDataQoS().get_rmw_qos_profile(), sub_opts2);

    sync_ = std::make_unique<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(50), *sub1_, *sub2_);
    sync_->registerCallback(std::bind(&PointCloudFusionNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    pub_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Publishing fused PointCloud2 to topic: %s", output_topic_.c_str());
  }

private:
  void callback(const PointCloud2::ConstSharedPtr a, const PointCloud2::ConstSharedPtr b) {
    pcl::PointCloud<pcl::PointXYZ> pa, pb, pb_tr, pfused;
    pcl::fromROSMsg(*a, pa);
    pcl::fromROSMsg(*b, pb);
    pcl::transformPointCloud(pb, pb_tr, tr_);
    pfused = pa;
    pfused += pb_tr;
    PointCloud2 out;
    pcl::toROSMsg(pfused, out);
    out.header.frame_id = output_frame_id_;
    out.header.stamp = (rclcpp::Time(a->header.stamp) > rclcpp::Time(b->header.stamp)) ? a->header.stamp : b->header.stamp;
    applySelfFilter(out);
    pub_->publish(out);
  }

  void applySelfFilter(PointCloud2& msg) {
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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

using sensor_msgs::msg::PointCloud2;

pcl::PointCloud<pcl::PointXYZ>::Ptr make_grid(int n, float step) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(n*n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      cloud->push_back(pcl::PointXYZ((i-(n/2))*step, (j-(n/2))*step, 0.0f));
  return cloud;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_cloud_publishers");
  std::string topic1 = node->declare_parameter<std::string>("topic1", "/pc1");
  std::string topic2 = node->declare_parameter<std::string>("topic2", "/pc2");
  std::vector<double> trv = node->declare_parameter<std::vector<double>>("tr_row_major", {
    1,0,0,0.1,
    0,1,0,0.0,
    0,0,1,0.0,
    0,0,0,1
  });
  Eigen::Matrix4f T;
  for (int i=0;i<16;++i) T(i/4,i%4) = static_cast<float>(trv[i]);
  auto pub1 = node->create_publisher<PointCloud2>(topic1, rclcpp::SensorDataQoS());
  auto pub2 = node->create_publisher<PointCloud2>(topic2, rclcpp::SensorDataQoS());
  auto base = make_grid(50, 0.02f);
  pcl::PointCloud<pcl::PointXYZ> moved;
  pcl::transformPointCloud(*base, moved, T);
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok()) {
    PointCloud2 m1, m2;
    pcl::toROSMsg(*base, m1);
    pcl::toROSMsg(moved, m2);
    m1.header.frame_id = "map";
    m2.header.frame_id = "map";
    auto now = node->now();
    m1.header.stamp = now;
    m2.header.stamp = now;
    pub1->publish(m1);
    pub2->publish(m2);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <atomic>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using rclcpp::executors::SingleThreadedExecutor;

// Convert yaw (rad) to quaternion (Z rotation only)
inline void set_yaw_quat(geometry_msgs::msg::PoseStamped &p, double yaw_rad) {
  double cy = std::cos(yaw_rad * 0.5);
  double sy = std::sin(yaw_rad * 0.5);
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.0;
  p.pose.orientation.z = sy;
  p.pose.orientation.w = cy;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("offboard_path_with_yaw");
  SingleThreadedExecutor exec; exec.add_node(node);

  // --- State & pose subscribers
  std::atomic<bool> connected{false};
  auto sub_state = node->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10, [&](const mavros_msgs::msg::State& s){ connected = s.connected; });

  std::atomic<bool> pose_ready{false};
  geometry_msgs::msg::PoseStamped last_pose;
  auto sub_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", 10,
    [&](const geometry_msgs::msg::PoseStamped& p){ last_pose = p; pose_ready = true; });

  // --- Publisher & services
  auto sp_pub  = node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
  auto cli_arm = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  auto cli_mode= node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  cli_arm->wait_for_service(5s);
  cli_mode->wait_for_service(5s);

  // --- Helpers
  rclcpp::Rate rate(20);

  auto make_pose = [&](double x, double y, double z, double yaw_rad){
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map"; // ENU
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    set_yaw_quat(p, yaw_rad);
    return p;
  };

  auto fly_to = [&](double x, double y, double z, double yaw_rad, double timeout_s)->bool {
    const double tol = 0.30;
    auto target = make_pose(x, y, z, yaw_rad);
    rclcpp::Time t_start = node->now();
    while (rclcpp::ok() && (node->now() - t_start).seconds() < timeout_s) {
      target.header.stamp = node->now();
      sp_pub->publish(target);
      exec.spin_some();
      if (pose_ready.load()) {
        double dx = last_pose.pose.position.x - x;
        double dy = last_pose.pose.position.y - y;
        double dz = last_pose.pose.position.z - z;
        if (dx*dx + dy*dy + dz*dz < tol*tol) return true;
      }
      rate.sleep();
    }
    return false;
  };

  // --- Wait for FCU
  auto t0 = node->now();
  while (rclcpp::ok() && !connected) {
    if ((node->now() - t0).seconds() > 10.0) {
      RCLCPP_ERROR(node->get_logger(), "Timeout waiting for FCU connection.");
      rclcpp::shutdown(); return 1;
    }
    exec.spin_some(); rclcpp::sleep_for(100ms);
  }
  RCLCPP_INFO(node->get_logger(), "FCU connected.");

  // Prime OFFBOARD for ~2 s
  auto hover = make_pose(0.0, 0.0, 5.0, 0.0);
  rclcpp::Time t_prime = node->now();
  while ((node->now() - t_prime).seconds() < 2.0 && rclcpp::ok()) {
    hover.header.stamp = node->now();
    sp_pub->publish(hover);
    exec.spin_some();
    rate.sleep();
  }

  // Arm
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    auto fut = cli_arm->async_send_request(req);
    if (exec.spin_until_future_complete(fut, 5s) != rclcpp::FutureReturnCode::SUCCESS || !fut.get()->success) {
      RCLCPP_ERROR(node->get_logger(), "Arming failed."); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Armed.");
  }

  // OFFBOARD
  {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0; req->custom_mode = "OFFBOARD";
    auto fut = cli_mode->async_send_request(req);
    if (exec.spin_until_future_complete(fut, 5s) != rclcpp::FutureReturnCode::SUCCESS || !fut.get()->mode_sent) {
      RCLCPP_ERROR(node->get_logger(), "Failed to set OFFBOARD."); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Mode set to OFFBOARD.");
  }

  // ---- PATH ----
  const double z_hold = 5.0;
  const double YAW_EAST  = 0.0;
  const double YAW_NORTH = M_PI_2;
  const double YAW_SOUTH = -M_PI_2;

  // Start at (0,0,5)
  (void)fly_to(0.0, 0.0, z_hold, YAW_EAST, 5.0);

  // 1) 3 m forward → (3,0,5), face East
  fly_to(3.0, 0.0, z_hold, YAW_EAST, 7.5);

  // 2) 12 m left → (3,12,5), face North
  fly_to(3.0, 12.0, z_hold, YAW_NORTH, 10.0);

  // 3) 5 m forward → (8,12,5), face East
  fly_to(8.0, 12.0, z_hold, YAW_EAST, 10.0);

  // 4) 14 m right → (8,-2,5), face South
  fly_to(8.0, -2.0, z_hold, YAW_SOUTH, 12.0);

  // 5) 14 m left → (8,12,5), face North
  fly_to(8.0, 12.0, z_hold, YAW_NORTH, 12.0);

  // 6) 8 m forward → (16,12,5), face East
  fly_to(16.0, 12.0, z_hold, YAW_EAST, 10.0);

  // 7) 12 m right → (16,0,5), face South
  fly_to(16.0, 0.0, z_hold, YAW_SOUTH, 10.0);

  // ---- LAND ----
  {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0; req->custom_mode = "AUTO.LAND";
    auto fut = cli_mode->async_send_request(req);
    if (exec.spin_until_future_complete(fut, 5s) == rclcpp::FutureReturnCode::SUCCESS && fut.get()->mode_sent) {
      RCLCPP_INFO(node->get_logger(), "Landing.");
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to set AUTO.LAND; keeping OFFBOARD hover.");
    }
  }

  rclcpp::shutdown();
  return 0;
}

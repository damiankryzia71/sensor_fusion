from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("sensor_fusion")

    return LaunchDescription([
        DeclareLaunchArgument("input_topic1", default_value="/pcd_camera"),
        DeclareLaunchArgument("input_topic2", default_value="/pcd_lidar"),
        DeclareLaunchArgument("output_topic", default_value="/pcd_fused"),
        DeclareLaunchArgument("tr_matrix_config_path", default_value=PathJoinSubstitution([pkg_share, "config", "TR_identity.yaml"])),
        DeclareLaunchArgument("odom_topic", default_value="/px4_odom"),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            output="screen",
            arguments=[
                "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "camera_bridge_qos.yaml"])
            ],
            remappings=[
                ("/depth_camera/points", LaunchConfiguration("input_topic1"))
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge",
            output="screen",
            arguments=[
                "/world/baylands/model/x500_vision_0/link/lidar_link/sensor/lidar_3d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "lidar_bridge_qos.yaml"])
            ],
            remappings=[
                ("/world/baylands/model/x500_vision_0/link/lidar_link/sensor/lidar_3d_sensor/scan/points", LaunchConfiguration("input_topic2"))
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="odom_bridge",
            output="screen",
            arguments=[
                "/model/x500_vision_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
            ],
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "odom_bridge_qos.yaml"])
            ],
            remappings=[
                ("/model/x500_vision_0/odometry", LaunchConfiguration("odom_topic"))
            ]
        ),

        Node(
            package="sensor_fusion",
            name="pcd_fusion_node",
            executable="pcd_fusion_node",
            output="screen",
            parameters=[
                {
                    "input_topic1": LaunchConfiguration("input_topic1"),
                    "input_topic2": LaunchConfiguration("input_topic2"),
                    "output_topic": LaunchConfiguration("output_topic"),
                    "tr_matrix_config_path": LaunchConfiguration("tr_matrix_config_path")
                }
            ]
        )
    ])
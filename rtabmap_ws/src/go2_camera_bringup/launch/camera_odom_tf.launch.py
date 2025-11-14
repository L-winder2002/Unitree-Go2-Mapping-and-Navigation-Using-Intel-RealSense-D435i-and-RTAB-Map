from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== 声明参数 =====
    in_odom_arg = DeclareLaunchArgument(
        "in_odom",
        default_value="/utlidar/robot_odom",
        description="外部里程计输入 topic"
    )

    out_odom_arg = DeclareLaunchArgument(
        "out_odom",
        default_value="/robot_odom_fixed",
        description="校时后的里程计输出 topic"
    )

    sync_mode_arg = DeclareLaunchArgument(
        "sync_mode",
        default_value="rgb",
        description="时间同步模式: now | rgb"
    )

    target_hz_arg = DeclareLaunchArgument(
        "target_hz",
        default_value="60.0",
        description="里程计输出限频 (0=不节流)"
    )

    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        # 注意：这里假设你已经有 /camera/aligned_depth_to_color/image_raw 或 /camera/color/image_raw
        # 按你的实际相机节点话题修改
        default_value="/camera/color/image_raw",
        description="RGB 图像话题(用于 rgb 模式对齐)"
    )

    # ===== 静态外参：base_link -> camera_link =====
    static_tf_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        output="screen",
        arguments=[
            # x y z qx qy qz qw parent child
            "0.2", "0.0", "0.15",
            "0.0", "0.0", "0.0", "1.0",
            "base_link", "camera_link",
        ],
    )

    # ===== 里程计时间同步节点 =====
    odom_sync = Node(
        package='go2_camera_bringup',          # 你的 odom_time_sync.py 所在包
        executable='odom_time_sync',          # setup.py 里 console_scripts 的名字
        name='odom_time_sync',
        output='screen',
        parameters=[{
            'in_odom':   LaunchConfiguration('in_odom'),
            'out_odom':  LaunchConfiguration('out_odom'),
            'sync_mode': LaunchConfiguration('sync_mode'),   # 'now' 或 'rgb'
            'target_hz': LaunchConfiguration('target_hz'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
        }],
    )

    # ===== 将同步后的里程计发布为 TF =====
    odom_tf = Node(
        package='go2_camera_bringup',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('out_odom'),  # 默认 /robot_odom_fixed
            'fallback_parent': 'odom',
            'fallback_child': 'base_link',
        }],
    )

    return LaunchDescription([
        in_odom_arg,
        out_odom_arg,
        sync_mode_arg,
        target_hz_arg,
        rgb_topic_arg,
        static_tf_cam,
        odom_sync,
        odom_tf,
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def _find_rs_launch():
    for pkg in ("realsense2_camera", "realsense_camera"):
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            continue
        path = os.path.join(share, "launch", "rs_launch.py")
        if os.path.exists(path):
            return pkg, path
    raise RuntimeError("未找到 rs_launch.py，请确认已安装 realsense2_camera 包。")

def generate_launch_description():
    # ===== 声明所有会用到的参数（修复 in_odom 等未声明报错） =====
    serial_no_arg = DeclareLaunchArgument("serial_no", default_value="", description="RealSense 序列号")
    in_odom_arg   = DeclareLaunchArgument("in_odom",   default_value="/utlidar/robot_odom", description="外部里程计输入 topic")
    out_odom_arg  = DeclareLaunchArgument("out_odom",  default_value="/robot_odom_fixed",   description="校时后的里程计输出 topic")
    sync_mode_arg = DeclareLaunchArgument("sync_mode", default_value="rgb", description="时间同步模式: now | rgb")
    target_hz_arg = DeclareLaunchArgument("target_hz", default_value="60.0", description="里程计输出限频 (0=不节流)")
    rgb_topic_arg = DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw", description="RGB 话题(用于 rgb 模式对齐)")

    serial_no  = LaunchConfiguration("serial_no")

    # RealSense 的USB后端（按需）
    env_usb = SetEnvironmentVariable(name="LIBRS_USE_USB_BACKEND", value="1")

    # ===== 静态外参：base_link -> camera_link =====
    # 注意：静态TF只给了 base_link->camera_link；光学坐标由 realsense 发布（publish_tf:=true）
    static_tf_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        output="screen",
        arguments=[
            # x y z qx qy qz qw parent child
            "0.0","0.0","0.15",  "0.0","0.0","0.0","1.0",  "base_link","camera_link"
        ],
    )

    # ===== RealSense 相机（对齐深度 + 设备内同步 + 发布 camera_* TF）=====
    _, rs_launch_path = _find_rs_launch()
    rs_args = {
        "serial_no": serial_no,
        "enable_color": "true",
        "enable_depth": "true",
        "align_depth": "true",         # ← 修正键名
        "enable_sync": "true",
        "publish_tf": "true",          # ← 开启以发布 camera_* -> *_optical_frame TF
        # IMU可选：现在关闭
        "enable_gyro": "false",
        "enable_accel": "false",
        "unite_imu_method": "copy",
        # 帧率与分辨率（可按需改）
        "rgb_camera.color_profile": "640,480,30",
        "depth_module.depth_profile": "640,480,30",
        # image_transport 不是此驱动通用参数，去掉以免无效参数告警
        # "image_transport": "raw",
    }
    rs_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments=rs_args.items(),
    )

    # ===== 里程计时间同步节点（把外部 /utlidar/robot_odom 时间校到相机时域）=====
    odom_sync = Node(
        package='go2_camera_bringup',       # 你放 odom_time_sync.py 的包名（已按你给的保持不变）
        executable='odom_time_sync',        # setup.py 里 console_scripts 的可执行名
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

    odom_tf = Node(
    package='go2_camera_bringup',
    executable='odom_to_tf',
    name='odom_to_tf',
    output='screen',
    parameters=[{
        'odom_topic': '/robot_odom_fixed',   # 订阅同步后的 odom
        'fallback_parent': 'odom',
        'fallback_child': 'base_link'
    }]
)


    return LaunchDescription([
        serial_no_arg, in_odom_arg, out_odom_arg, sync_mode_arg, target_hz_arg, rgb_topic_arg,
        env_usb,
        static_tf_cam,
        rs_include,
        odom_sync,
        odom_tf
    ])

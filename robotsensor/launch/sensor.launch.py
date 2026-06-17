import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("robotsensor")

    urdf_path  = os.path.join(pkg, "urdf",   "robot.urdf.xacro")
    rviz_path  = os.path.join(pkg, "config",  "robot_car.rviz")
    world_path = os.path.join(pkg, "worlds",  "world.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_path])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--verbose", "-r", world_path],
        output="screen",
        additional_env={
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins",
            "IGN_GAZEBO_RESOURCE_PATH": "/opt/ros/humble/share",
        },
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  "robot_car",
            "-topic", "/robot_description",
            "-z",     "0.15",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            # cmd_vel goes to diff_drive_controller
            "/diff_drive_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )

    load_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "arm_controller"],
        output="screen",
    )

    load_drive = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "diff_drive_controller"],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    set_arm_zero = ExecuteProcess(
    cmd=["ros2", "topic", "pub", "--once",
         "/arm_controller/joint_trajectory",
         "trajectory_msgs/msg/JointTrajectory",
         "{joint_names: ['arm_joint1'], points: [{positions: [0.0], time_from_start: {sec: 1}}]}"],
    output="screen",
)



    return LaunchDescription([
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    rsp,
    gazebo,
    TimerAction(period=8.0,  actions=[spawn]),       
    TimerAction(period=12.0, actions=[bridges]),      
    TimerAction(period=15.0, actions=[load_jsb, load_arm, load_drive]),
    TimerAction(period=16.0, actions=[set_arm_zero]),
    TimerAction(period=17.0, actions=[rviz2]),        
])
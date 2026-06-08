import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    robot_desc = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }]
    )

    
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    
    spawn_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                name='spawn_rover',
                output='screen',
                arguments=[
                    '-name',  'rover',
                    '-topic', '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.15',
                ]
            )
        ]
    )

   
    bridge_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_ign_bridge',
                executable='parameter_bridge',
                name='ign_ros2_bridge',
                output='screen',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/world/default/model/rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                ],
                remappings=[
                    ('/world/default/model/rover/joint_state', '/joint_states'),
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        jsp_node,
        gazebo,
        bridge_node,
        spawn_node,
    ])
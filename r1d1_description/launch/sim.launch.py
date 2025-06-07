from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    share_dir = get_package_share_directory('r1d1_description')

    # Process the XACRO file to get the URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'r1d1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    config_file = os.path.join(
        get_package_share_directory('r1d1_description'),
        'config',
        'ignition.yaml'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "ign_args":"-r"
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-file", xacro_file,
            "-x", "0",
            "-y", "0",
            "-z", "0"
        ],
        output="screen"
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={config_file}'
        ],
        output='screen'
    )

    ros2_config = PathJoinSubstitution([
        FindPackageShare("r1d1_description"),
        "config",
        "r1d1.yaml",
    ])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_config],
        output='screen'
    )

    diff_cont = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    
    

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_launch,
        spawn_robot,
        gz_bridge_node,
        controller_manager_node,
        joint_state_broadcaster,
        diff_cont,
        
    ])

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('r1d1_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'r1d1_m.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    urdf_path = os.path.join(share_dir, 'urdf', 'r1d1_m.urdf')
    world = os.path.join(share_dir,'world','world_with_aruco.world')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(robot_urdf)
    print("URDF file generated successfully")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': use_sim_time,}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'use_sim_time': 'true',
            'world': world
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'r1d1',
            '-topic', 'robot_description'
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster", "-c", "/controller_manager"
        ],
        parameters=[
            {'use_sim_time': True}
        ],
    )
    slider_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "slider_controller", "-c", "/controller_manager"
        ]
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "arm_controller", "-c", "/controller_manager"
        ]
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "gripper_controller", "-c", "/controller_manager"
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        joint_state_broadcaster_spawner,
        slider_controller_spawner,
        gripper_controller_spawner,
        arm_controller_spawner,
    ])
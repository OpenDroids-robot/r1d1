import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    share_dir = get_package_share_directory('r1d1_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    xacro_file = os.path.join(share_dir, 'urdf', 'r1d1_m.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    gazebo_params_file = os.path.join(share_dir,'config','gazebo_params.yaml')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output = 'screen',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': use_sim_time,
            }
        ],
        arguments=[robot_urdf]
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
            # 'pause': 'true',
            'use_sim_time': 'true'
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
    execute_process = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen'
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
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "arm_controller", "-c", "/controller_manager"
        ]
    )

    return LaunchDescription([
        
        gazebo_server,
        gazebo_client,
        # execute_process,
        urdf_spawn_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # diff_controller_spawner,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])

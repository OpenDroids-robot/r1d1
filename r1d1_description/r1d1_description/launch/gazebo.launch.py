from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('r1d1_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'r1d1_m.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    urdf_path = os.path.join(share_dir, 'urdf', 'r1d1.urdf')
    world = os.path.join(share_dir,'world','empty_world.sdf')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(robot_urdf)
    print("URDF file generated successfully")

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

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -s ', world],
            'pause': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-g '}.items()
    )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'r1d1',
            '-file', urdf_path,
            '-topic', 'robot_description'
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            #'/lidar_data@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            #'/lidar_data/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "arm_controller", "-c", "/controller_manager"
        ]
    )
    slider_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "slider_controller", "-c", "/controller_manager"
        ]
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "gripper_controller", "-c", "/controller_manager"
        ]
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

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        slider_controller_spawner,
        gripper_controller_spawner
    ])

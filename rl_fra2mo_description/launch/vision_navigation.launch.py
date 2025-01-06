from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction





def generate_launch_description():
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("rl_fra2mo_description"),
                "launch",
                "fra2mo_explore.launch.py"
            ])
        ])
    )


    follow_waypoint_node = Node(
        package='rl_fra2mo_description',
        executable='follow_waypoints.py',
        parameters=[{"target": "aruco"}],  # Specifica il parametro target
        output='screen'
    )

    posebroadcaster_node = Node(
        package='rl_fra2mo_description',
        executable='PoseToTFBroadcaster.py',
        output='screen'
    )



    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'marker_id',
            default_value='115',
            description='ID del marker ArUco da rilevare.'
        ),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.1',
            description='Dimensione del marker ArUco in metri.'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/videocamera',
            description='Topic della fotocamera da usare per il rilevamento.'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='Topic delle informazioni della fotocamera.'
        ),
        DeclareLaunchArgument(
            'marker_frame', 
            default_value='aruco_marker_frame',
            description='Frame ID associato al marker.'
        ),
        DeclareLaunchArgument(
            'reference_frame', 
            default_value='map',
            description='Frame di riferimento per la posa calcolata.'
        ),
        DeclareLaunchArgument(
            'corner_refinement', 
            default_value='LINES',
            description='Metodo di affinamento degli angoli del marker.'
        ),
        DeclareLaunchArgument(
            'camera_frame', 
            default_value='camera_link_optical',
            description=''
        ),
    
    ]

    # Node configuration
    aruco_single_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': 'map',
            'camera_frame': 'camera_link_optical',
            'marker_frame': LaunchConfiguration('marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
            # 'use_sim_time': True
        }],
        remappings=[
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/image', LaunchConfiguration('camera_topic')),
        ]
    )
            
    
    return LaunchDescription([
        *declared_arguments,
        explore_launch,
        follow_waypoint_node,
        aruco_single_node,
        posebroadcaster_node
    ])

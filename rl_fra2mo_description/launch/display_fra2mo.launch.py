import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Definisci un argomento di lancio
    rviz_arg = DeclareLaunchArgument(
        "map",  # Nome dell'argomento
        default_value="false",  # Valore predefinito
        description="Choose RViz configuration: 'nav' for conf_robot.rviz, 'map' for navigation.rviz"
    )

    # Ottieni il valore dell'argomento
    # rviz_config = LaunchConfiguration("rviz_config")


    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"

    rviz_nav_config_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'rviz_conf', 'conf_robot.rviz')
    rviz_map_config_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'rviz_conf', 'navigation.rviz')
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)
    
    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # # Nodo RViz2
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # Nodo RViz per configurazione 'nav'
    rviz_nav_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_nav_config_file],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('map'))
    )

    # Nodo RViz per configurazione 'map'
    rviz_map_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_map_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('map'))
    )

    # nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    nodes_to_start = [rviz_arg, robot_state_publisher_node, joint_state_publisher_node, rviz_map_node, rviz_nav_node]

    return LaunchDescription(nodes_to_start)

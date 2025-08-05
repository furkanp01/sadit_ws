# sadit_description/launch/gazebo.launch.py
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sadit_description = get_package_share_directory('sadit_description')

    # Dünya dosyasının tam yolunu belirle
    world = os.path.join(pkg_sadit_description, 'world', 'sadit.world')

    # Gazebo sunucusunu (fizik motoru) kendi dünyamızla başlat
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'pause': 'false'}.items()
    )

    # Gazebo arayüzünü başlat
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robotu Gazebo'ya ekle (spawn et)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sadit', '-x', '0.0', '-y', '0.0', '-z' '0.5'],
        output='screen'
    )
    
    # Sadece robotun durumunu yayınlayan düğüm. Kontrolcü değil.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # robot_description'ı doğrudan vermek yerine, bir üst launch'tan gelmesini beklemek
        # daha modülerdir ama bu basit yapı için bu şekilde kalabilir.
        parameters=[{'robot_description': xacro.process_file(os.path.join(pkg_sadit_description, 'urdf', 'sadit.xacro')).toxml()}]
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity,
    ])
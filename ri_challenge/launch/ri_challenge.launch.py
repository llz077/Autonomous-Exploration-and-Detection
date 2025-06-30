from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
import time
 
def generate_launch_description():
    # Create launch actions
    start_yolo_detector = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yolo_detector'), 'launch', 'yolo_detector.launch.py')))

    start_alert_player = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('alert_player'), 'launch', 'alert_player.launch.py')))
    
    start_ri_challenge_node = Node(
            package='ri_challenge',
            #executable='ri_challenge_node',
            executable='ri_challenge_node',
            name='ri_challenge_node',
            output="screen",
            emulate_tty=True)
    
    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(start_yolo_detector)
    ld.add_action(start_alert_player)
    ld.add_action(start_ri_challenge_node)
    
    return ld

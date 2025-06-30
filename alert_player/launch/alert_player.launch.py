from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    # Create launch actions
    start_alert_player = Node(
            package='alert_player',
            executable='alert_player',
            name='alert_player',
            #remappings=[('/original_topic', '/remapped_topic')],
            output="screen",
            emulate_tty=True)
    
    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(start_alert_player)
    
    return ld

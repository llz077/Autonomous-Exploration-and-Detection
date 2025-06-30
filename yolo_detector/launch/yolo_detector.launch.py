from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    # Create launch actions
    start_yolo_node = Node(
            package='yolo_detector',
            executable='yolo_node',
            name='yolo_detector_node',
            remappings=[('/image_input_topic', '/camera/image_raw'),
                        ('/image_input_topic/compressed', '/camera/image_raw/compressed')],
            output="screen",
            parameters=[{'rate_limit': 2.0}, 
                        {'compressed_input': True}],
            emulate_tty=True)
    
    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(start_yolo_node)
    
    return ld

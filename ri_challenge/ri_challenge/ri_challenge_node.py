import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from yolo_detector_msgs.msg import BoundingBox
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import threading
from collections import deque
import time

############################ TO BE IMPLEMENTED ###############################
#### Location Definitions ####
locations = {}
locations["starting_pos"] = [0.0, 0.0, 0.0, 0.000, 0.000, 0.000, 1.000]
locations["room1_entrance"] = [0.828, -1.364, 0.010, 0.000, 0.000, 0.016, 1.000]
locations["room1_west"] = [1.935, -0.810, 0.010, 0.000, 0.000, 0.000, 1.000]
locations["room1_south"] = [1.935, -0.810, 0.010, 0.000, 0.000, 0.705, 0.709]
locations["room1_north"] = [1.933, -0.810, 0.010, 0.000, 0.000, -0.697, 0.717]
locations["room2_entrance"] = [0.729, -3.022, 0.010, 0.000, 0.000, -0.012, 1.000]
locations["room2_west"] = [1.913, -2.748, 0.010, 0.000, 0.000, 0.010, 1.000]
locations["room2_south"] = [1.915, -2.747, 0.010, 0.000, 0.000, 0.713, 0.701]
locations["room2_north"] = [1.913, -2.735, 0.010, 0.000, 0.000, -0.669, 0.743]
##############################################################################
 
class RI_Challenge(Node):
    def __init__(self):
        super().__init__('ri_challenge')
        self.get_logger().info("Starting Robotics Intelligence Challenge!")

        # Create Alert Message Publisher
        self.alert_publisher_ = self.create_publisher(String, '/alert', 10)
        # Create Object Detection Subscribers
        self.bounding_box_subscriber = self.create_subscription(
            BoundingBox, '/bounding_boxes', self.bounding_box_callback, 10)
        self.detection_image_subscriber = self.create_subscription(
            CompressedImage, '/image_output_topic/compressed', self.detection_output_image_callback, 10)
        
        # Create Navigation Action Client
        self.navigator = BasicNavigator()
        self.set_initial_pose(locations["starting_pos"])
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Navigation Stack Ready!")

        # Object detection parameters
        self.detection_queue = deque()
        self.detection_frame_count = 0
        self.detection_label = ""

        # Main thread
        self.main_thread = threading.Thread(target=self.main_thread, daemon=True)
        self.main_thread.start()    # Start Robot Actions
 
    def main_thread(self):
        # Send "start" notification
        self.get_logger().info("Starting Challenge")
        self.send_alert("started")
        challenge_start_time = time.time()

        ############################ TO BE IMPLEMENTED ###############################
        ######## Example : Go to a location in Room1 and check for intruders #########
        
        # Go to room1
        
        # stop_detected = self.detect('stop sign', frame_count=4, min_detections=1)
        # if (stop_detected):
        #     self.send_alert("Closed room") # Send "intruder" notification
            
        self.get_logger().info("Go To Room1 Entrance")
        self.navigate_to_target(locations["room1_entrance"])

        stop_detected = self.detect('stop sign', frame_count=4, min_detections=1)
        if (stop_detected):
            self.send_alert("Closed room") # Send "intruder" notification
        else:
            self.scan_room1()
        
        # Go to room2
        self.get_logger().info("Go To Room2 Entrance")
        self.navigate_to_target(locations["room2_entrance"])

        stop_detected = self.detect('stop sign', frame_count=4, min_detections=1)
        if (stop_detected):
            self.send_alert("Closed room") # Send "intruder" notification
        else:
            self.scan_room2()


        ##############################################################################
        self.get_logger().info("Go Back To Starting Position")
        self.navigate_to_target(locations["starting_pos"])

        self.get_logger().info("Challenge Complete!!")
        self.get_logger().info("Total Time: %d seconds"%(time.time() - challenge_start_time))

    '''
    Performs object detection

    Checks if a given 'label' is detected in camera images.
    Waits until a given 'frame_count' is processed or timeout occurs

    Returns:
        True - If more than min_detections are detected within the frame_count
        False - otherwise
    '''
    def detect(self, label, frame_count=1, min_detections=1, timeout=5.0):
        # Set label
        self.detection_label = label

        # Reset counters
        self.detection_queue.clear()
        self.detection_frame_count = 0
        self.detection_start_time = time.time()

        while self.detection_frame_count < frame_count:
            if (timeout < (time.time() -self.detection_start_time)):
                self.get_logger().warn("Detection Timeout: [label, frame_count, min_detections] : %s, %d, %d"%(
                    label, frame_count, min_detections))
                return False
            
            # Rate control
            time.sleep(0.1)
        
        if len(self.detection_queue) >= min_detections:
            self.get_logger().info("Detected: %s"%label)
            return True
        else:
            return False
    
    def detection_output_image_callback(self, msg):
        self.detection_frame_count += 1

    def bounding_box_callback(self, msg):
        # If received label doesnt match current detection label, ignore
        if self.detection_label != msg.label:
            return
        
        if msg.y < (msg.image_height/4):
            return
        ################### TO BE IMPLEMENTED #####################
        # Perform any filtering based on bounding box placement
        # (To filter out background detections)
        # E.g. Ignore bounding box, if box center is higher than
        #          image horizontal center-line.
        # if (msg.y / msg.height) > 0.5:
        #     return
        ############################################################

        self.detection_queue.append(msg)

    '''
    Initialize the robot start position in the map
    Arguments : pose
                [x, y, z, qx, qy, qz, qw]

    Note: The input argument should be list or a tuple, 
            defining the target position and orientation in map
    '''
    def set_initial_pose(self, pose):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = pose[0]
        initial_pose.pose.position.y = pose[1]
        initial_pose.pose.position.z = pose[2]
        initial_pose.pose.orientation.x = pose[3]
        initial_pose.pose.orientation.y = pose[4]
        initial_pose.pose.orientation.z = pose[5]
        initial_pose.pose.orientation.w = pose[6]

        self.get_logger().info("Setting Initial Pose!")
        self.navigator.setInitialPose(initial_pose)
    
    '''
    Navigate the robot to a given target
    Arguments : pose
                [x, y, z, qx, qy, qz, qw]

    Note: The input argument should be list or a tuple, 
            defining the target position and orientation
    '''
    def navigate_to_target(self, pose, timeout=60.0):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]
        goal_pose.pose.orientation.x = pose[3]
        goal_pose.pose.orientation.y = pose[4]
        goal_pose.pose.orientation.z = pose[5]
        goal_pose.pose.orientation.w = pose[6]
        
        self.get_logger().info("Navigating to : [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]"%(
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]))
        self.navigator.goToPose(goal_pose)

        self.wait_until_navigation_complete(timeout=timeout)
    '''
    Waits until Navigation is complete or timeout occurs.
    Default timeout value to 60.0

    Returns:
        True - Navigation Complete
        False - Error / Timeout
    '''
    def wait_until_navigation_complete(self, timeout=30.0):
        while not self.navigator.isTaskComplete():
            # If you want, you can do something with the feedback
            feedback = self.navigator.getFeedback()

            # Check timeout
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=timeout):
                self.navigator.cancelTask()
            
            # Rate control to stop spamming action server
            time.sleep(0.5)

        # Check status
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation Done!")
            return True
        else:
            print('Navigation Failed!')
            return False

    def send_alert(self, msg):
        alert_msg = String()
        alert_msg.data = msg
        self.alert_publisher_.publish(alert_msg)

    def scan_room1(self):
        self.get_logger().info("Go To Room1 South Wall")
        self.navigate_to_target(locations["room1_south"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification
        
        self.get_logger().info("Go To Room1 West Wall")
        self.navigate_to_target(locations["room1_west"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification
            
        self.get_logger().info("Go To Room1 North Wall")
        self.navigate_to_target(locations["room1_north"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification

    def scan_room2(self):
        self.get_logger().info("Go To Room2 West Wall")
        self.navigate_to_target(locations["room2_west"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification
        
        self.get_logger().info("Go To Room2 South Wall")
        self.navigate_to_target(locations["room2_south"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification
            
        self.get_logger().info("Go To Room2 North Wall")
        self.navigate_to_target(locations["room2_north"])
        # Check for intruders
        intruder_detected = self.detect('person', frame_count=4, min_detections=2)
        if (intruder_detected):
            self.send_alert("intruder") # Send "intruder" notification

def main(args=None):
    rclpy.init(args=args)
 
    ri_challenge = RI_Challenge()
    
    executor = MultiThreadedExecutor()
    executor.add_node(ri_challenge)

    try:
        executor.spin()
    finally:
        ri_challenge.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound
 
from ament_index_python.packages import get_package_share_directory
from collections import deque
from threading import Thread
import os
import time
 
class AlertPlayer(Node):
 
    def __init__(self):
        super().__init__('alert_player')
        
        # Create Alert Queue & Alert History
        self.alert_queue = deque()
        self.alert_history = {}

        # Get package installation directory
        self.pkg_dir = get_package_share_directory("alert_player")
        # Media directory (Within installation directory)
        self.media_dir = os.path.join(self.pkg_dir, 'media')
        
        # Create Subscriber
        self.subscriber = self.create_subscription(String, '/alert', self.alert_callback, 10)
        # Create playback thread
        self.playback_thread = Thread(target=self.playback, daemon=True)
        self.playback_thread.start()
 
        self.get_logger().info("Alert Player Node Started!")
    
    def __del__(self):
        print (self.alert_history)

    def playback(self):
        while rclpy.ok():
            if len(self.alert_queue) > 0:
                cmd = self.alert_queue.pop()
                alert_file = os.path.join(self.media_dir, cmd + '.mp3')
                if os.path.exists(alert_file):
                    # Record History
                    if cmd in self.alert_history:
                        self.alert_history[cmd] += 1
                    else:
                        self.alert_history[cmd] = 1
                    # Show history
                    self.get_logger().info(cmd + " : Total Count : " + str(self.alert_history[cmd]))
                else:
                    self.get_logger().warn("Alert Phrase Error. Received : " + cmd)
                    alert_file = os.path.join(self.media_dir, 'wrong input.mp3')
                
                # Play Alert
                playsound(alert_file)

            # Loop rate limiter (10Hz)
            time.sleep(0.1)

    def alert_callback(self, msg):
        alert_phrase = msg.data.lower()
        self.alert_queue.append(alert_phrase)
 
def main(args=None):
    rclpy.init(args=args)
 
    alert_player = AlertPlayer()
 
    rclpy.spin(alert_player)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
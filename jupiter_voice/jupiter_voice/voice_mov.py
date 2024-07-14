#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class MoveBase(Node):

    def __init__(self):
        super().__init__('move_base_node')
        self.get_logger().info("move_base node started.")
        # Create subscriber and publisher
        self.voice_cmd_sub = self.create_subscription(String, "/voice_cmd", self.move_base_callback, 10)
        self.publisher = self.create_publisher(String, "/voice_tts", 10)

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Basic Nav is activated.")
    
    def move_base_callback (self, msg):
        text = msg.data
        if 'study' in text.lower():
            self.move_to_goal(1.0, 0.0, 0.0) # location of study on the map
        elif 'kitchen' in text.lower():
            self.move_to_goal(8.0, -1.5, 0.0) # location of the kitchen on the map
        elif 'passage' in text.lower():
            self.move_to_goal(2.5, 0.0, 3.14) # location of the passage on the map
        elif 'tv' in text.lower():
            self.move_to_goal(3.0, -6.0, 0.0) # location of the tv on the map
        elif 'piano' in text.lower():
            self.move_to_goal(9.0, -7.0, 0.0) # location of the piano on the map
        elif 'bedroom two' in text.lower():
            self.move_to_goal(10.0, -11.0, 0.0) # location of the bed-2 on the map

    def move_to_goal(self, pos_x, pos_y, rot_z):
        try:
            goal_pose = self.create_pose_stamped(self.navigator, pos_x, pos_y, rot_z)
            # --- Going to one pose ---
            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                #self.get_logger().info(str(feedback))              
        except Exception as e:
            self.get_logger().error(f'Request error: {e}')
            self.send_voice_tts('Destination unreachable')
        finally:
            self.get_logger().info(str(self.navigator.getResult()))
            self.send_voice_tts('Destination reached')
        
                            
    def create_pose_stamped(self, navigator: BasicNavigator, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose
    
    # send the message to convert text to voice via the /voice_tts topic
    def send_voice_tts(self, text):
        tts_msg = String()
        tts_msg.data = text
        self.publisher.publish(tts_msg)
        #self.get_logger().info(text)
        

def main(args=None):
    rclpy.init(args=args)
    node = MoveBase()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

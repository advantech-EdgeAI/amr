import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SpeechListener(Node):
    def __init__(self):
        super().__init__('speech_listener')
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("goal_text_topic", "/goal_response")

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self.posestamped_callback,
            10
        )
        self.goal_text_subscriber = self.create_subscription(
            String,
            self.get_parameter("goal_text_topic").value,
            self.string_callback,
            10
        )
        self.logger = self.get_logger()
        self.logger.info(f"Listener Node is ready.")


    def posestamped_callback(self, msg):
        self.get_logger().info(f'Goal pose: "{msg.pose}"')

    def string_callback(self, msg):
        self.get_logger().info(f'Goal text: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
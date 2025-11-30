import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatesToCmdBridge(Node):
    def __init__(self):
        super().__init__('joint_states_to_command_bridge_for_isaacsim')

        # Subscribe to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        # Publish to /joint_command_isaac
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_command_isaac',
            10)

    def listener_callback(self, msg):
        # Publish the received JointState message to /joint_command_isaac
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_states_to_cmd_bridge = JointStatesToCmdBridge()

    try:
        rclpy.spin(joint_states_to_cmd_bridge)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    joint_states_to_cmd_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

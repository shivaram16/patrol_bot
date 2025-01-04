#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,Twist

class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        self.twist_sub = self.create_subscription(Twist,"/mux_twist_out/cmd_vel",self.TwistSubCallback,10)
        self.twist_stamped_out_pub = self.create_publisher(TwistStamped,"/diff_drive_controller/cmd_vel",10)

    def TwistSubCallback(self,msg):

        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg
        self.twist_stamped_out_pub.publish(twist_stamped_msg)

    
def main():
    rclpy.init()
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
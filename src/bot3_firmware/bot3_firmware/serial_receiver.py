#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialReceiver(Node):

    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.declare_parameter("port","/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").to_parameter_msg()._value.integer_value

        self.pub = self.create_publisher(String,"serial_receiver",10)
        self.freq = 0.01
        self.arduino = serial.Serial(port=self.port,baudrate=self.baudrate)
        self.timer = self.create_timer(self.freq,self.msg_payload)

    def msg_payload(self):
        if rclpy.ok() and self.arduino.is_open:
            data = self.arduino.readline()
            try:
                data.decode("utf-8")
            except:
                return
            
        msg = String()
        msg.data = str(data)
        self.pub.publish(msg)

        self.get_logger().info("recieved info : %s" %(msg.data))


def main():
        rclpy.init()
        simple_serial_receiver = SimpleSerialReceiver()
        rclpy.spin(simple_serial_receiver)
        simple_serial_receiver.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
        
    main()
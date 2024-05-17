#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import States,TorqueInput

class pendulum_interface(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscriber = self.create_subscription(States,"/state_feedback",self.callback1,10)
        self.publisher = self.create_publisher(TorqueInput,"/torque_input",10)
        self.timer = self.create_timer(1/3000,self.send_torque)
    def callback1(self,msg:States):
        self.get_logger().info("theta: "+ str(msg.theta))
        self.get_logger().info("ang_vel: " + str(msg.theta_dot))
    
    def send_torque(self):
        t = TorqueInput()
        t.torque_value = 1.0
        self.publisher.publish(t)
    

def main(args=None):
    rclpy.init(args=args)
    node = pendulum_interface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

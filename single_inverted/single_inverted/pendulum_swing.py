#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from custom_msgs.msg import States,TorqueInput
import matplotlib.pyplot as plt
import time
class pendulum_swing(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscriber = self.create_subscription(States,"/state_feedback",self.callback1,10)
        self.publisher = self.create_publisher(TorqueInput,"/torque_input",10)
        #self.timer = self.create_timer(4,self.send_torque)
        self.theta = 0 - (np.random.rand() - 0.5) / 2
        self.theta = (self.theta + np.pi)%(2*np.pi) - np.pi
        self.theta_dot = 0
        self.torque = 0
        self.setpoint  = np.pi
        
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001
        self.integral = 0
        self.previous_error = self.setpoint - self.theta
        self.previous_theta = 0
        self.torque_prev = 5.0


        self.Kp= 50
        self.Ki = 0
        self.Kd = 6

        self.theta_values = []
        self.time_values = []

        self.swing_up_completed = False


    def callback1(self,msg:States):
        self.theta = msg.theta
        self.theta_dot = msg.theta_dot
        t = TorqueInput()
        if not self.swing_up_completed:
            if abs(self.theta) < 2*np.pi/3:
                if abs(self.theta - self.previous_theta) > 0.1:
                    t.torque_value =0.0
                if abs(self.theta - self.previous_theta) < 0.001:
                    self.torque_prev *= -1
                    t.torque_value = self.previous_theta
            if abs(self.theta)>2*np.pi/3:
                self.swing_up_completed = True
            

        # PID controller for stabilization
        error = (np.pi - self.theta) if self.theta >= 0 else -(np.pi + self.theta)
        p = self.Kp * error
        dt = time.time() - self.t_prev
        self.t_prev = time.time()
        self.integral += error * dt
        i = self.Ki * self.integral
        d = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error
        t.torque_value = p + i + d

        # Clip the torque to the maximum allowed value
        t.torque_value = max(min(t.torque_value, 5.0), -5.0)
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = pendulum_swing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

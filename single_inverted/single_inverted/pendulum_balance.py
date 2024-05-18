#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from custom_msgs.msg import States,TorqueInput
import matplotlib.pyplot as plt
import time
class pendulum_balance(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscriber = self.create_subscription(States,"/state_feedback",self.callback1,10)
        self.publisher = self.create_publisher(TorqueInput,"/torque_input",10)
        #self.timer = self.create_timer(4,self.send_torque)
        self.theta = np.pi - (np.random.rand() - 0.5) / 2
        self.theta = (self.theta + np.pi)%(2*np.pi) - np.pi
        self.theta_dot = 0
        self.torque = 0
        self.setpoint  = np.pi
        
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001
        self.integral = 0
        self.previous_error = self.setpoint - self.theta


        self.Kp= 50
        self.Ki = 0
        self.Kd = 0.6

        self.theta_values = []
        self.time_values = []


    def callback1(self,msg:States):
        self.theta = msg.theta
        self.theta_dot = msg.theta_dot

        t = TorqueInput()
        if self.theta>=0:
            error = (np.pi - self.theta)
        if self.theta<0:
            error = -(np.pi + self.theta)

        p = self.Kp * error
        dt = 1/500
        self.t_prev = time.time()
        self.integral += error* dt
        i = self.Ki * self.integral
        d = self.Kd * (error - self.previous_error)/dt
        self.previous_error = error
        t.torque_value = (p + i + d)
        if t.torque_value>5:
            t.torque_value = 5.0
        if t.torque_value<-5:
            t.torque_value = -5.0
        self.publisher.publish(t)
        self.get_logger().info("theta: "+ str(self.theta))
        #self.get_logger().info("theta_dot: "+ str(self.theta_dot))
        self.theta_values.append(self.theta)
        self.time_values.append(time.time() - self.t_start)

    def plot_theta_vs_time(self):
        plt.plot(self.time_values, self.theta_values)
        plt.xlabel('Time (s)')
        plt.ylabel('Theta (rad)')
        plt.title('Theta vs Time')
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = pendulum_balance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_theta_vs_time()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

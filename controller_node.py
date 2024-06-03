#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from custom_msgs.msg import States
from custom_msgs.msg import TorqueInput
import matplotlib.pyplot as plt
import time

# Create a list to store the theta and time values
theta_values = []
time_values = []

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.controller_node_subs = self.create_subscription(States, "/state_feedback", self.pose_callback, 10) # create a subscriber with the name controller_node_subs that subscribes to the state_feedback topic
        self.controller_node_pub = self.create_publisher(TorqueInput, "/torque_input", 10) # create a publisher with the name controller_node_pub that publishes to the torque_input topic
        #self.timer = self.create_timer(0.02, self.send_torque_info) # create a timer that calls the send_torque_info function every 0.01 seconds
        #Set the state variables
        self.theta = np.pi - (np.random.rand() - 0.5) / 2
        self.theta = (self.theta + np.pi)%(2*np.pi) - np.pi
        self.theta_dot = 0
        self.torque = 0.0
        self.target_angle = np.pi

        self.start_time = time.time()
        self.prev_time = self.start_time
        self.previous_error = self.target_angle - self.theta
        self.integral = 0

        self.get_logger().info("Controller node has been started")

        #Inititalize lists to store theta and time values
        


    def pose_callback(self, msg: States):
        #self.get_logger().info("Received state: " + str(msg.theta) + ", " + str(msg.theta_dot))
        self.theta = msg.theta
        self.theta_dot = msg.theta_dot

        if self.theta < 0:
            error = -self.target_angle - self.theta
        if self.theta >= 0:
            error = self.target_angle - self.theta

        self.PID(error)
        theta_values.append(self.theta) # append the theta value to the list
        time_values.append(time.time() - self.start_time) # append the time value to the list
        

    def send_torque_info(self): # create a function that sends the torque value to the simulation

        self.torque = max(-5.0, min(5.0, self.torque)) # limit the torque value to be between -5 and 5

        #Publish the torque value
        msg = TorqueInput() # create a TorqueInput message
        msg.torque_value = self.torque
        self.controller_node_pub.publish(msg) # publish the torque value to the topic
        self.get_logger().info("Sent torque: " + str(self.torque)) # log the torque value


    def PID(self,error):

        # Set the PID gains
        Kp = 50
        Ki = 0
        Kd = 0.5

        # Compute the time difference
        dt = time.time() - self.prev_time
        self.prev_time = time.time() # update the previous time

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        p = Kp * error
        i = Ki * self.integral
        d = Kd * derivative

        self.torque = p + i + d
        self.send_torque_info()


def plot_graph(theta_values, time_values):   
    
    plt.plot(time_values, theta_values)
    plt.xlabel("Time (s)")
    plt.ylabel("Theta (rad)")
    plt.title("Theta vs Time")
    plt.grid(True)
    plt.show()   


        




def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()
    rclpy.spin(node)

    plot_graph(theta_values, time_values)
    
    rclpy.shutdown()



   
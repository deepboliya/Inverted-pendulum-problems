import numpy as np
import matplotlib.pyplot as plt
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs.msg import TorqueInput, States

class Double_Inverted_Pendulum(Node):

    # Initialise States : 
    theta_1_init = np.pi - (np.random.rand() - 0.5) / 2
    theta_2_init = np.pi / 3

    theta1_dot_init = 0.0
    theta2_dot_init = 0.0

    # Param/s
    mass1 = 1.0 # grams
    mass2 = 0.1 # grams
    l1 = 1.0 # metres
    l2 = 1.0 # metres
    a = 0.05 # metres
    g = 9.81 # m / sec ^ 2

    state_update_frequency = 500
    state_update_timeperiod = 1 / state_update_frequency

    feedback_frequency = 50

    def __init__ (self) : 
        super().__init__('main')

        # Timers : 
        update_states_timer = self.create_timer(1 / self.state_update_frequency, self.Update_Pendulum_States)
        feedback_timer = self.create_timer(1 / self.feedback_frequency, self.Feedback)

        # Publishers : 
        self.visualizer1 = self.create_publisher(Marker, '/pendulum_viz1', 1)
        self.visualizer2 = self.create_publisher(Marker, '/pendulum_viz2', 1)

        self.feedback_pub1 = self.create_publisher(States, '/state_feedback1', 1)
        self.feedback_pub2 = self.create_publisher(States, '/state_feedback2', 1)

        # Attributes : 
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001
        self.obj_id = 0

        # States : 
        self.theta1 = self.theta_1_init
        self.theta2 = self.theta_2_init

        self.theta1_dot = self.theta1_dot_init
        self.theta2_dot = self.theta2_dot_init
    
    def Update_Pendulum_States(self) :
        dt = time.time() - self.t_prev
        self.t_prev = time.time()

        #Calculation of angular accelerations :)
        Theta1_DoubleDot = (-self.g * (2 * self.mass1 + self.mass2) * np.sin(self.theta1)
                     - self.mass2 * self.g * np.sin(self.theta1 - 2 * self.theta2)
                     - 2 * np.sin(self.theta1 - self.theta2) * self.mass2 * (self.theta2_dot**2 * self.l2 + self.theta1_dot**2 * self.l1 * np.cos(self.theta1 - self.theta2))) / (self.l1 * (2 * self.mass1 + self.mass2 - self.mass2 * np.cos(2 * self.theta1 - 2 * self.theta2)))

        Theta2_DoubleDot = (2 * np.sin(self.theta1 - self.theta2) * 
                     (self.theta1_dot**2 * self.l1 * (self.mass1 + self.mass2) 
                      + self.g * (self.mass1 + self.mass2) * np.cos(self.theta1) 
                      + self.theta2_dot**2 * self.l2 * self.mass2 * np.cos(self.theta1 - self.theta2))) / (self.l2 * (2 * self.mass1 + self.mass2 - self.mass2 * np.cos(2 * self.theta1 - 2 * self.theta2)))

        # Incrementing omegas
        self.theta1_dot += Theta1_DoubleDot * dt
        self.theta2_dot += Theta2_DoubleDot * dt

        # Incrementing thetas
        self.theta1 += self.theta1_dot * dt
        self.theta2 += self.theta2_dot * dt

        self.visualise_pendulum()

        return
    
    def Feedback(self):
        states_msg1 = States()
        states_msg1.theta = self.theta1
        states_msg1.theta_dot = self.theta1_dot

        states_msg2 = States()
        states_msg2.theta = self.theta2
        states_msg2.theta_dot = self.theta2_dot

        self.feedback_pub1.publish(states_msg1)
        self.feedback_pub2.publish(states_msg2)

        return


    def visualise_pendulum(self):
        pendulum_marker1 = Marker()
        pendulum_marker1.header.frame_id = "map"
        pendulum_marker1.id = self.obj_id
        pendulum_marker1.type = Marker.LINE_STRIP
        pendulum_marker1.action = Marker.ADD
        pendulum_marker1.pose.orientation.w = 1.0
        pendulum_marker1.scale.x = 0.05  # Line width

        # Set points
        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = - self.l1 * cos(self.theta1)
        point_2.z = 0.0
        pendulum_marker1.points = [point_1,
                            point_2
                        ]
        
        pendulum_marker2 = Marker()
        pendulum_marker2.header.frame_id = "map"
        pendulum_marker2.id = self.obj_id
        pendulum_marker2.type = Marker.LINE_STRIP
        pendulum_marker2.action = Marker.ADD
        pendulum_marker2.pose.orientation.w = 1.0
        pendulum_marker2.scale.x = 0.05  # Line width

        point_3 = Point()
        point_3.x = point_2.x + (self.l2 * sin(self.theta2))
        point_3.y = point_2.y + (self.l2 * cos(self.theta2))
        point_3.z = 0.0
        pendulum_marker2.points = [point_2,
                            point_3
                        ]
        pendulum_marker1.color.b = 1.0
        pendulum_marker1.color.g = 1.0
        pendulum_marker1.color.a = 1.0  # Alpha value
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker1.lifetime = Duration_of_pendulum_marker  # Permanent pendulum_marker
        self.visualizer1.publish(pendulum_marker1)

        pendulum_marker2.color.g = 1.0
        pendulum_marker1.color.r = 1.0
        pendulum_marker2.color.a = 1.0  # Alpha value
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker2.lifetime = Duration_of_pendulum_marker  # Permanent pendulum_marker
        self.visualizer2.publish(pendulum_marker2)

        self.obj_id += 1

def main(args = None):

    rclpy.init(args = args)
    pendulum = Double_Inverted_Pendulum()
    rclpy.spin(pendulum)

    pendulum.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

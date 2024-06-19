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

class double_inverted_pendulum(Node):

    # Initialize State/s : at theta = pi/2, pi/2
    theta1_0 = np.pi / 2
    theta2_0 = np.pi / 2
    theta1_dot0 = 0.0
    theta2_dot0 = 0.0
    
    # You can use similar variations as the original code for initialization

    
    #theta is almost upright (randomized)
    theta1_0 = np.pi - (np.random.rand() - 0.5) / 2
    theta1_0 = (theta1_0 + np.pi)%(2*np.pi) - np.pi

    theta2_0 = np.pi - (np.random.rand() - 0.5) / 2
    theta2_0 = (theta2_0 + np.pi)%(2*np.pi) - np.pi

    theta1_dot0 = 0
    theta2_dot0 = 0
    
    
    # Input/s
    torque_value = 0.0

    # Param/s
    mass1 = 1.0
    mass2 = 1.0
    g = 9.81
    l1 = 1.0
    l2 = 1.0
    state_update_frequency = 500
    state_update_timeperiod = 1 / state_update_frequency

    feedback_frequency = 50

    def __init__(self):
        super().__init__('double_pendulum')

        # Timers
        self.update_states_timer = self.create_timer(1 / self.state_update_frequency, self.update_pendulum_states)
        #self.feedback_timer = self.create_timer(1 / self.feedback_frequency, self.feedback)

        # Publishers/Subscribers/Services
        self.visualizer = self.create_publisher(Marker, '/pendulum_viz', 1)
        self.feedback_pub = self.create_publisher(States, '/state_feedback', 1)
        self.input = self.create_subscription(TorqueInput, '/torque_input', self.update_input_torque, 5)

        # Attributes
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001
        self.obj_id = 0

        # States
        self.theta1 = self.theta1_0
        self.theta2 = self.theta2_0
        self.theta1_dot = self.theta1_dot0
        self.theta2_dot = self.theta2_dot0

        self.get_logger().info('Double Inverted Pendulum node initialized')
        self.get_logger().info('Accepting Input')
        self.get_logger().info('Publishing Feedback')

    def f(self, x, u):
        m1, m2, l1, l2, g = self.mass1, self.mass2, self.l1, self.l2, self.g
        theta1, theta2, theta1_dot, theta2_dot = x

        delta = theta2 - theta1
        denom1 = (m1 + m2) * l1 - m2 * l1 * cos(delta) ** 2
        denom2 = (l2 / l1) * denom1

        theta1_ddot = (m2 * l1 * theta1_dot ** 2 * sin(delta) * cos(delta) +
                       m2 * g * sin(theta2) * cos(delta) +
                       m2 * l2 * theta2_dot ** 2 * sin(delta) -
                       (m1 + m2) * g * sin(theta1)) / denom1

        theta2_ddot = (-m2 * l2 * theta2_dot ** 2 * sin(delta) * cos(delta) +
                       (m1 + m2) * g * sin(theta1) * cos(delta) -
                       (m1 + m2) * l1 * theta1_dot ** 2 * sin(delta) -
                       (m1 + m2) * g * sin(theta2)) / denom2

        return np.array([theta1_dot, theta2_dot, theta1_ddot, theta2_ddot])

    def update_pendulum_states(self):
        dt = time.time() - self.t_prev
        self.t_prev = time.time()

        x = np.array([self.theta1, self.theta2, self.theta1_dot, self.theta2_dot])
        x_intermediate = x + 0.5 * dt * self.f(x, self.torque_value)
        x += dt * self.f(x_intermediate, self.torque_value)

        self.theta1, self.theta2, self.theta1_dot, self.theta2_dot = x

        self.visualize_pendulum()
        return

    '''
    def feedback(self):
        states_msg = States()
        states_msg.theta1 = self.theta1
        states_msg.theta2 = self.theta2
        states_msg.theta1_dot = self.theta1_dot
        states_msg.theta2_dot = self.theta2_dot
        self.feedback_pub.publish(states_msg)
        return
    '''
    
    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = self.obj_id
        pendulum_marker.type = Marker.LINE_STRIP
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05  # Line width

        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = - self.l1 * cos(self.theta1)
        point_2.z = 0.0

        point_3 = Point()
        point_3.x = point_2.x + self.l2 * sin(self.theta2)
        point_3.y = point_2.y - self.l2 * cos(self.theta2)
        point_3.z = 0.0

        pendulum_marker.points = [point_1, point_2, point_3]

        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  # Alpha value
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker

        self.visualizer.publish(pendulum_marker)

        self.obj_id += 1

    def update_input_torque(self, msg):
        self.torque_value = max(-5, min(5, msg.torque_value))
        return

def main(args=None):
    rclpy.init(args=args)
    pendulum_ = double_inverted_pendulum()
    rclpy.spin(pendulum_)

    pendulum_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import numpy as np
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs.msg import  States

class double_pendulum(Node):
    theta0_1 = np.pi + 0.01  
    theta_dot0_1 = 0.0
    theta0_2 = np.pi/4 # pi, -pi represents upright
    theta_dot0_2 = 0.0
    
    state_update_frequency = 3000
    state_update_timeperiod = 1 / state_update_frequency

    feedback_frequency = 50

    def __init__(self):
        super().__init__('main')
        self.m1 = 1.0
        self.m2= 1.0
        self.g = 9.81
        self.l1= 1.0
        self.l2=1.0

        update_states_timer = self.create_timer(1 / self.state_update_frequency, self.update_pendulum_states)
        feedback_timer = self.create_timer(1 / self.feedback_frequency, self.feedback)
        self.visualizer1 = self.create_publisher(Marker, '/pendulum_viz1', 1)
        self.visualizer2 = self.create_publisher(Marker, '/pendulum_viz2', 1)
        self.feedback_pub1 = self.create_publisher(States, '/state_feedback1', 1)
        self.feedback_pub2 = self.create_publisher(States, '/state_feedback2', 1)

        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001   
        self.obj_id = 0

        self.theta1 = self.theta0_1
        self.omega1 = self.theta_dot0_1
        self.theta2 = self.theta0_2
        self.omega2 = self.theta_dot0_2
        self.get_logger().info('Double Inverted Pendulum node initialized')
        self.get_logger().info('Accepting Input')
        self.get_logger().info('Publishing Feedback')

    def update_pendulum_states(self):

        dt = time.time() - self.t_prev
        self.t_prev = time.time()

        domega1_dt = (-self.g * (2 * self.m1 + self.m2) * np.sin(self.theta1) - self.m2 * self.g * np.sin(self.theta1 - 2 * self.theta2) - 2 * np.sin(self.theta1 - self.theta2) * self.m2 * (self.omega2 ** 2 * self.l2 + self.omega1 ** 2 * self.l1 * np.cos(self.theta1 - self.theta2))) / (self.l1 * (2 * self.m1 + self.m2 - self.m2 * np.cos(2 * self.theta1 - 2 * self.theta2)))
        domega2_dt = (2 * np.sin(self.theta1 - self.theta2) * (self.omega1 ** 2 * self.l1 * (self.m1 + self.m2) + self.g * (self.m1 + self.m2) * np.cos(self.theta1) + self.omega2 ** 2 * self.l2 * self.m2 * np.cos(self.theta1 - self.theta2))) / (self.l2 * (2 * self.m1 + self.m2 - self.m2 * np.cos(2 * self.theta1 - 2 * self.theta2)))


        self.omega1 += domega1_dt * dt
        self.omega2 += domega2_dt * dt


        self.theta1 += self.omega1 * dt
        self.theta2 += self.omega2 * dt

        self.visualize_pendulum()
       
        return

    def feedback(self):
        states_msg1 = States()
        states_msg1.theta = self.theta1
        states_msg1.theta_dot = self.omega1
        states_msg2 = States()
        states_msg2.theta = self.theta2
        states_msg2.theta_dot = self.omega2
        self.feedback_pub1.publish(states_msg1)
        self.feedback_pub2.publish(states_msg2)
        return
    
    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = 0
        pendulum_marker.type = Marker.LINE_STRIP
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05


        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = - self.l1 * cos(self.theta1)
        point_2.z = 0.0
        pendulum_marker.points = [point_1,
                            point_2
                        ]

        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker  
        self.visualizer1.publish(pendulum_marker)

        pendulum_marker2 = Marker()
        pendulum_marker2.header.frame_id = "map"
        pendulum_marker2.id = 1
        pendulum_marker2.type = Marker.LINE_STRIP
        pendulum_marker2.action = Marker.ADD
        pendulum_marker2.pose.orientation.w = 1.0
        pendulum_marker2.scale.x = 0.05 

        point_3 = Point()
        point_3.x = point_2.x+(self.l2 * sin(self.theta2))
        point_3.y =  point_2.y-(self.l2 * cos(self.theta2))
        point_3.z = 0.0
        pendulum_marker2.points = [point_2,
                            point_3
                        ]

        pendulum_marker2.color.r = 1.0
        pendulum_marker2.color.a = 1.0  
        Duration_of_pendulum_marker2 = Duration()
        Duration_of_pendulum_marker2.sec = 0
        Duration_of_pendulum_marker2.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker2.lifetime = Duration_of_pendulum_marker2  
        self.visualizer2.publish(pendulum_marker2)

        self.obj_id += 1

    

def main(args = None):

    rclpy.init(args = args)
    pendulum_ = double_pendulum()
    rclpy.spin(pendulum_)
    pendulum_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
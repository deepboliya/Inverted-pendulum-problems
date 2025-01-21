import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from math import pi
import matplotlib.pyplot as plt
from custom_msgs.msg import States, TorqueInput

class Single_Inverted(Node):

    # Param/s
    mass = 1.0
    g = 9.81
    l = 1.0
    state_update_frequency = 3000
    state_update_timeperiod = 1 / state_update_frequency

    feedback_frequency = 50
    # feedback_timeperiod = 1 / feedback_frequency

    inertia = mass * l * l

    def __init__(self):
        super().__init__('Pendulum_Node')
        self.subscription = self.create_subscription(
            States,
            '/state_feedback',
            self.talker_callback,
            10
        )

        self.publisher = self.create_publisher(
            TorqueInput,
            '/torque_input',
            10
        )
    
    def talker_callback(self, msg: States):
        Theta_Meas = abs(msg.theta)

        # Publish the control torque
        Torque_msg = TorqueInput()

        if Theta_Meas > 0 : Torque_msg.torque_value = 1.0 * self.inertia
        else : Torque_msg.torque_value = -1.0
        
        self.publisher.publish(Torque_msg)

def main(args=None):
    rclpy.init(args=args)
    
    Pendulum_Node = Single_Inverted()
    rclpy.spin(Pendulum_Node)

    Pendulum_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


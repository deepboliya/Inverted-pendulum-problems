from typing import List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import time

from custom_msgs.msg import States, TorqueInput

class Single_Inverted_Swing_Up(Node):
    
    # Setting Limits
    Target_Value = np.pi
    Torque_Max = 5.0
    
    # Setting terms for errors
    Theta_prev = 0.0
    Torque_prev = Torque_Max
    
    # Setting terms for PID
    prev_error = 0.0
    Integral_Total = 0.0

    def __init__(self):
        super().__init__("swing_up_pendulum")

        self.create_subscription(
            States, 
            "/state_feedback",
            self.handle_state_msg,
            10
        )
        self.torque_publisher = self.create_publisher(
            TorqueInput,
            "/torque_input",
            10
        )

        self.prev_time = time.time()

    def handle_state_msg(self, msg: States):
        Theta_meas = abs(msg.theta)
        curr_time = time.time()

        Torque_msg = TorqueInput()
        error = self.Target_Value - Theta_meas

        Kp = 0.15
        Ki = 0.002
        Kd = 4.0
        
        # If the chnage i angle is greater than 0.1 radian, Troque = 0
        if abs(Theta_meas - self.Theta_prev) >= 0.1:
            Torque_msg.torque_value = 0.0
        
        # If absolute angle is less than 135 deg, and change in angle is less than 0.001 rad, Reverse Torque 
        elif abs(Theta_meas) < 3 * np.pi / 4:
            if abs(Theta_meas - self.Theta_prev) < 0.001:
                self.Torque_prev *= -1

            Torque_msg.torque_value = self.Torque_prev
        
        # Else PID :)
        else:
            dt = curr_time - self.prev_time

            self.Integral_Total += error
            derivative = (error - self.prev_error) / dt

            PIDvalue = Kp * error + Ki * self.Integral_Total + Kd * derivative

            Torque_msg.torque_value = Theta_meas + PIDvalue
            if msg.theta < 0:
                Torque_msg.torque_value *= -1

        self.torque_publisher.publish(Torque_msg)

        # Set all current values to previous for next error calculations
        self.Theta_prev = Theta_meas
        self.prev_error = error
        self.prev_time = curr_time


def main(args=None):
    rclpy.init(args=args)

    node = Single_Inverted_Swing_Up()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

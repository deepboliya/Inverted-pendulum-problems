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

        # PID gains
        self.kp = 0.15
        self.ki = 0.002
        self.kd = 1.5

        # Initialize PID controller state
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.prev_time = self.get_clock().now()

        # Initialize lists to store theta and time data for plotting
        self.theta_data = []
        self.time_data = []
    
    def Time_Diff(self, t1: Time, t2: Time) -> float:
        t1_msg = t1.to_msg()
        t2_msg = t2.to_msg()
        sec_diff = t1_msg.sec - t2_msg.sec
        nanosec_diff = t1_msg.nanosec - t2_msg.nanosec
        return sec_diff + nanosec_diff / 1e9
    
    def talker_callback(self, msg: States):
        Theta_Meas = abs(msg.theta)
        curr_time = self.get_clock().now()

        # Compute error terms
        error = np.pi - Theta_Meas
        self.integral_error += error
        self.derivative_error = (error - self.previous_error) / self.Time_Diff(curr_time, self.prev_time)

        # Compute the control torque
        control_torque = (
            self.kp * error
            + self.ki * self.integral_error
            + self.kd * self.derivative_error
        )
        
        # Updating errors 
        self.previous_error = error
        self.prev_time = curr_time

        # Store theta and time data for plotting
        self.theta_data.append(Theta_Meas)
        self.time_data.append(curr_time.seconds_nanoseconds()[0] + curr_time.seconds_nanoseconds()[1] / 1e9)

        # Publish the control torque
        Torque_msg = TorqueInput()

        Torque_msg.torque_value = Theta_Meas + control_torque
        if msg.theta < 0 : Torque_msg.torque_value *= -1
        
        self.publisher.publish(Torque_msg)

    def plot_theta_vs_time(self):
        plt.plot(self.time_data, self.theta_data, label='Theta vs Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Theta (radians)')
        plt.title('Theta vs Time')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    
    Pendulum_Node = Single_Inverted()
    try:
        rclpy.spin(Pendulum_Node)
    except KeyboardInterrupt:
        pass

    # Plot theta vs time after shutting down the node
    Pendulum_Node.plot_theta_vs_time()

    Pendulum_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


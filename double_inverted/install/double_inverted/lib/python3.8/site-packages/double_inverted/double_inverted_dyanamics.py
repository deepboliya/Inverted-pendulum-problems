import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class DoubleInvertedPendulumSimulator(Node):
    def __init__(self):
        super().__init__("double_inverted_pendulum_simulator")

        # Initialize pendulum parameters
        self.length1 = 1.0  # Length of the first pendulum
        self.length2 = 1.0  # Length of the second pendulum
        self.mass1 = 1.0    # Mass of the first pendulum
        self.mass2 = 1.0    # Mass of the second pendulum
        self.gravity = 9.81 # Acceleration due to gravity

        # Initial state [theta1, omega1, theta2, omega2]
        self.state = np.array([np.pi / 2, 0, np.pi / 2, 0])

        # Time step for the simulation
        self.dt = 0.01

        # Setup the figure and axis for visualization
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.grid()

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.show()

    def equations_of_motion(self, state):
        theta1, omega1, theta2, omega2 = state

        delta = theta2 - theta1

        denom1 = (self.mass1 + self.mass2) * self.length1 - self.mass2 * self.length1 * np.cos(delta) * np.cos(delta)
        denom2 = (self.length2 / self.length1) * denom1

        a1 = (self.mass2 * self.length1 * omega1 * omega1 * np.sin(delta) * np.cos(delta) +
              self.mass2 * self.gravity * np.sin(theta2) * np.cos(delta) +
              self.mass2 * self.length2 * omega2 * omega2 * np.sin(delta) -
              (self.mass1 + self.mass2) * self.gravity * np.sin(theta1)) / denom1

        a2 = (- self.mass2 * self.length2 * omega2 * omega2 * np.sin(delta) * np.cos(delta) +
              (self.mass1 + self.mass2) * self.gravity * np.sin(theta1) * np.cos(delta) -
              (self.mass1 + self.mass2) * self.length1 * omega1 * omega1 * np.sin(delta) -
              (self.mass1 + self.mass2) * self.gravity * np.sin(theta2)) / denom2

        return np.array([omega1, a1, omega2, a2])

    def step(self):
        self.state += self.equations_of_motion(self.state) * self.dt

    def update_plot(self, frame):
        self.step()

        theta1, _, theta2, _ = self.state

        x1 = self.length1 * np.sin(theta1)
        y1 = -self.length1 * np.cos(theta1)
        x2 = x1 + self.length2 * np.sin(theta2)
        y2 = y1 - self.length2 * np.cos(theta2)

        self.line.set_data([0, x1, x2], [0, y1, y2])
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = DoubleInvertedPendulumSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time

plt.style.use('seaborn-pastel')


class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/force_torque_sensor/readings',
            self.listener_callback,
            1000)
        self.times = []
        self.voltages = []
        self.subscription  # prevent unused variable warning
        self.start = time.time()
        # TODO: change figure settings
        # fig,ax=plt.subplots()
        # ax.scatter(self.times,self.voltages)
        # fig.canvas.mpl_connect('button_press_event', self.listener_callback)
        # plt.show()
        # plt.draw()
        plt.ion()
        # self.fig = plt.figure()
        # ax = self.fig.add_subplot(111)
        # self.line1, = ax.plot(self.times, self.voltages, 'b-')

    def listener_callback(self, msg):
        curr = time.time()
        self.times.append(curr - self.start)
        self.voltages.append(msg.data[0])

        # plt.clf()
        if len(self.times) > 50:
            self.times.pop(0)
            self.voltages.pop(0)
        plt.plot(self.times, self.voltages, 'b-')
        plt.show()
        plt.pause(.001)
        


def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
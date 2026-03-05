import rclpy
import random
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollowerLogger(Node):
# TODO: wallfollower metrics - lateral error to wall e(t) = d_desired - d_measured
# => show an oscillation plot of e(t) + some post run metrics like avg like avg amplitude + frequency

# TODO: control + smoothness metrics - the actual messages being logged to the car
# => plot the mean change in steering angle deltas 1/(n-1) sum (delta U)/(delta t) where U = U_t - U_t-1
# => find maximum spike max(delta U)

# TODO: safety controller metrics
# => detected angle
# => minimum predicted TTC

    def __init__(self):
        super().__init__('')
        self.declare_parameters(namespace = '', parameters = [
            ('sc_drive_points', 'logger/obstacles'), # safety controller obstacles topic
            ('wf_drive_topic', ''), # wall follower drive topic
            ('sc_drive_topic',''), # safety controller drive topic
            ('jerkiness_topic','logger/jerkiness'), # wall follower jerkiness topic
            ('lateral_error_topic','logger/wf_error'), # wall follower lateral error topic (i might combine into one message with the topic given above)
        ])

        self.WF_TOPIC = self.get_parameter('wf_drive_topic').get_parameter_value().string_value
        self.SC_TOPIC = self.get_parameter('sc_drive_topic').get_parameter_value().string_value
        self.OBSTACLES_TOPIC = self.get_parameter('sc_drive_points').get_parameter_value().string_value
        self.JERKINESS_TOPIC = self.get_parameter('jerkiness_topic').get_parameter_value().string_value
        self.ERROR_TOPIC = self.get_parameter('jerkiness_topic').get_parameter_value().string_value

        self.SC_points_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.SC_TOPIC,
            self.SC_points_callback,
            10)

        self.WF_drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.WF_TOPIC,
            self.WF_drive_callback,
            10)

        self.WF_error_subscription = self.create_subscription(
            None,
            self.ERROR_TOPIC,
            self.error_callback,
            10)

        rclpy.get_shutdown_context().on_shutdown(self.on_shutdown_callback)

        self.avg_distances = []
        self.recent_points = None
        self.min_crash_margin = float("infinity")
        self.crashes_detected = 0.0

    def on_shutdown_callback(self):
        """
        This node gets called when the run is finished, i.e. the logger node is shut down.
        """
        self.get_logger().info("--- Node is shutting down. Saving a log of the run. ---")
        distance_time_axis = [0 + i for i in range(len(self.avg_distances))]

        fig2 = plt.figure()
        plt.plot(distance_time_axis,self.avg_distances)
        plt.figure('Average Distance To Obstacles During the Run')
        plt.ylabel('Distance (m)')
        plt.xlable('Timestep')

        fig2.savefig('avg_dist')
        fig2.close()
        min_dist = min(self.avg_distances)

        self.get_logger().info(f'Minimum crash margin: {min_dist}/')

        distances_df = pd.DataFrame(self.avg_distances)
        distances_df.to_csv('avg_dist',index=False)
        self.get_logger().info(f'Saved csv files and plots to {os.getcwd()}/')

    def SC_points_callback(self, msg):
        """
        Docstring for SC_points_callback

        :param self: Description
        :param msg: Description
        """
        points = msg.ranges
        self.recent_points = points
        n = len(points)
        if n > 0:
            self.get_logger().info(f'Currently detecting a crash of size: {n}')
            self.crashes_detected += 1

        avg_distance = np.mean(np.array(n))

        self.obstacle_sizes.append(n)
        self.avg_distances.append(avg_distance)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WallFollowerLogger()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry

import numpy as np
import math


class ViveOdomEvaluator(Node):
    GRID_SPACING = 0.5
    GRID_SIZE = (5, 5)
    MAX_COUNT_PER_CELL = 500
    SETTLE_COUNT = 1000

    def __init__(self):
        super().__init__('vive_odom_evaluator')
        self.subscription = self.create_subscription(Odometry,
                                                     '/tracker',
                                                     self.vive_msg_cb,
                                                     qos_profile_sensor_data)

        self.grid = np.zeros(self.GRID_SIZE)
        self.grid_counts = np.zeros(self.GRID_SIZE)
        self.settle = 0
        self.last_cell = (None, None)

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print("\nZ Averages: \n", self.grid / self.grid_counts)
        print("Counts: \n", self.grid_counts, "\n")

    def position_to_grid_cell(self, pose):
        y = math.floor(pose.y / self.GRID_SPACING)
        x = math.floor(pose.x / self.GRID_SPACING)
        return x, y

    def cell_in_bounds(self, cell):
        return cell[0] >= 0 and cell[1] >= 0 and not (cell[0] >= self.GRID_SIZE[0] or cell[1] >= self.GRID_SIZE[1])

    def vive_msg_cb(self, msg):
        pose = msg.pose.pose.position
        cell = self.position_to_grid_cell(pose)
        # cell == last cell?
        if cell != self.last_cell:
            self.settle = 0
            self.last_cell = cell
            if not self.cell_in_bounds(cell):
                print(f"Tracker out of bounds in cell: {cell}")
            else:
                print(f"Tracker is in cell: {cell}")
        elif self.settle < self.SETTLE_COUNT:
            self.settle += 1
        elif self.cell_in_bounds(cell) and self.grid_counts[cell[0], cell[1]] < self.MAX_COUNT_PER_CELL:
            self.grid[cell[0], cell[1]] += pose.z
            self.grid_counts[cell[0], cell[1]] += 1


def main(args=None):
    rclpy.init(args=args)
    vive_odom_evaluator = ViveOdomEvaluator()
    rclpy.spin(vive_odom_evaluator)
    vive_odom_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

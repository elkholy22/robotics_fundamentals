#!/usr/bin/env python3
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from map_and_messages.msg import Cell, Grid, Row


class MapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("example_map_publisher")
        self.publisher = self.create_publisher(Grid, "map", 1)
        self.grid = self._build_grid()
        self.timer = self.create_timer(1.0, self._publish_grid)
        self.get_logger().info(f"Publishing map: {self.grid}")

    def _build_grid(self) -> Grid:
        T = Cell.TOP
        B = Cell.BOTTOM
        L = Cell.LEFT
        R = Cell.RIGHT

        package_share = get_package_share_directory("map_and_messages")
        filename = os.path.join(package_share, "map.txt")
        with open(filename, "r", encoding="utf-8") as map_file:
            map_source = map_file.read()

        namespace = {"T": T, "B": B, "L": L, "R": R}
        exec(f"map_layout = {map_source}", {}, namespace)
        map_layout = namespace["map_layout"]

        grid = Grid()
        for row_layout in map_layout:
            row = Row()
            for cell_layout in row_layout:
                cell = Cell()
                cell.walls = cell_layout
                row.cells.append(cell)
            grid.rows.append(row)

        return grid

    def _publish_grid(self) -> None:
        self.publisher.publish(self.grid)


def main() -> None:
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from create_fundamentals.msg import SensorPacket
from create_fundamentals.srv import DiffDrive, Leds, PlaySong, ResetEncoders, StoreSong
from irobot.create import Create


class CreateDriver(Node):
    def __init__(self):
        super().__init__("create")
        port = self.declare_parameter("port", "/dev/ttyUSB0").get_parameter_value().string_value
        baud = self.declare_parameter("baud", 57600).get_parameter_value().integer_value
        self.get_logger().info(f"Initializing iCreate driver on port: {port} @ {baud} baud")
        self.create = Create(port, baud=baud)
        self.get_logger().info("Serial connection opened and robot reset completed")

        self.packet_publisher = self.create_publisher(SensorPacket, "sensor_packet", 1)
        self.fields = [
            "wheeldrop_caster",
            "wheeldrop_left",
            "wheeldrop_right",
            "bump_left",
            "bump_right",
            "wall",
            "cliff_left",
            "cliff_front_left",
            "cliff_front_right",
            "cliff_right",
            "virtual_wall",
            "infrared_byte",
            "advance",
            "play",
            "encoder_left",
            "encoder_right",
            "charging_state",
            "voltage",
            "current",
            "battery_temperature",
            "battery_charge",
            "battery_capacity",
            "wall_signal",
            "cliff_left_signal",
            "cliff_front_left_signal",
            "cliff_front_right_signal",
            "cliff_right_signal",
            "home_base",
            "internal_charger",
            "song_number",
            "song_playing",
        ]
        self.create.update = self.sense

        self.create_service(ResetEncoders, "reset_encoders", self.reset_encoders_callback)
        self.create_service(Leds, "leds", self.leds_callback)
        self.create_service(StoreSong, "store_song", self.store_song_callback)
        self.create_service(PlaySong, "play_song", self.play_song_callback)
        self.create_service(DiffDrive, "diff_drive", self.diff_drive_callback)

        self.get_logger().info("ROS services created: reset_encoders, leds, store_song, play_song, diff_drive")

    def start(self):
        self.get_logger().info("Sending Create start/full-mode sequence")
        self.create.start()
        self.get_logger().info("Create monitor thread running")

    def stop(self):
        self.get_logger().info("Stopping Create driver")
        self.create.stop()

    def sense(self):
        packet = SensorPacket()
        packet.header.stamp = self.get_clock().now().to_msg()
        for field in self.fields:
            try:
                setattr(packet, field, getattr(self.create, field))
            except AttributeError:
                return
        self.packet_publisher.publish(packet)

    def leds_callback(self, req, res):
        self.create.leds(req.play, req.advance, req.color, req.intensity)
        res.success = True
        return res

    def diff_drive_callback(self, req, res):
        self.get_logger().info(f"diff_drive request: left={req.left:.3f}, right={req.right:.3f}")
        self.create.diffDrive(req.left, req.right)
        res.success = True
        return res

    def reset_encoders_callback(self, req, res):
        _ = req
        self.create.clear()
        res.success = True
        return res

    def store_song_callback(self, req, res):
        self.get_logger().info(f"store_song request: number={req.number}, bytes={len(req.song)}")
        self.create.storeSong(req.number, *req.song)
        res.success = True
        return res

    def play_song_callback(self, req, res):
        self.get_logger().info(f"play_song request: number={req.number}")
        self.create.playSong(req.number)
        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)
    node = CreateDriver()
    sleep(1.0)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

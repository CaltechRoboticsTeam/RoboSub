"""Read Teledyne PD0 from the DVL serial adapter."""

import fcntl
import json
import os
import termios
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from robosub_control.drivers.teledyne_pd0 import Pd0Decoder, rotate_velocity
from robosub_control.utils.tuning import fixed_parameter


def open_serial(device, baud):
    rate = getattr(termios, f"B{baud}")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attributes = termios.tcgetattr(fd)
        attributes[:4] = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0]
        attributes[4:6] = [rate, rate]
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
    except BaseException:
        os.close(fd)
        raise
    return fd


class TeledyneDvl(Node):
    def __init__(self, **kwargs):
        super().__init__("dvl_reader", **kwargs)
        self.device = fixed_parameter(self, "device", "/dev/ttyUSB0")
        self.baud = fixed_parameter(self, "baud", 115200)
        self.start_stream = fixed_parameter(self, "start_stream", True)
        self.mounting_confirmed = fixed_parameter(
            self, "mounting_confirmed", False
        )
        self.rpy = fixed_parameter(
            self, "instrument_to_body_rpy_deg", [0.0, 90.0, 0.0]
        )
        self.velocity_sign = fixed_parameter(self, "velocity_sign", 1.0)
        self.fd = None
        self.decoder = Pd0Decoder()
        self.last_packet = None
        self.last_sample = None
        self.last_published = None
        self.next_open = 0.0
        self.error = "Waiting for the serial device"
        self.pending_start = b""
        self.last_ensemble = None
        self.velocity_pub = self.create_publisher(
            Odometry, "/sub/sensors/dvl/odometry", 10
        )
        self.status_pub = self.create_publisher(
            String, "/sub/sensors/dvl/status", 10
        )
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.periodic, clock=clock)
        self.create_timer(0.5, self.publish_status, clock=clock)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def periodic(self):
        now = time.monotonic()
        try:
            if self.fd is None:
                if now < self.next_open:
                    return
                self.next_open = now + 2.0
                self.fd = open_serial(self.device, self.baud)
                self.decoder = Pd0Decoder()
                self.last_packet = None
                self.last_sample = None
                self.last_published = None
                self.last_ensemble = None
                self.pending_start = b"CS\r" if self.start_stream else b""
                self.error = "Serial open; waiting for checksum-valid PD0"
                self.get_logger().info(
                    f"DVL serial {self.device} at {self.baud} baud; "
                    "startup sends CS only, with no reset or saved settings."
                )
            if not os.path.exists(self.device):
                raise ConnectionError("DVL serial device disappeared")
            if self.pending_start:
                try:
                    sent = os.write(self.fd, self.pending_start)
                    if sent == 0:
                        raise ConnectionError(
                            "DVL start command made no progress"
                        )
                    self.pending_start = self.pending_start[sent:]
                except BlockingIOError:
                    return
            # Bound each callback so noisy input cannot starve ROS callbacks.
            for _ in range(4):
                try:
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    break
                if not data:
                    break
                for sample in self.decoder.feed(data):
                    self.on_sample(sample)
        except OSError as error:
            self.close()
            self.error = str(error)
            self.get_logger().warning(self.error, throttle_duration_sec=2.0)

    def on_sample(self, sample):
        self.last_packet = time.monotonic()
        self.last_sample = sample
        self.error = sample["error"]
        if sample["velocity"] is None:
            return
        ensemble = sample.get("ensemble")
        if ensemble is None or ensemble == self.last_ensemble:
            self.error = "Missing or repeated PD0 ensemble counter"
            return
        self.last_ensemble = ensemble
        if not self.mounting_confirmed:
            self.error = "PD0 bottom track received; mounting/sign unverified"
            return
        velocity = rotate_velocity(
            sample["velocity"], self.rpy, self.velocity_sign
        )
        output = Odometry()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
        output.pose.covariance[0] = -1.0
        output.twist.twist.linear.x = velocity[0]
        output.twist.twist.linear.y = velocity[1]
        output.twist.twist.linear.z = velocity[2]
        for index in (0, 7, 14):
            output.twist.covariance[index] = 0.02
        self.velocity_pub.publish(output)
        self.last_published = time.monotonic()

    def publish_status(self):
        now = time.monotonic()
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "device": self.device,
                        "baud": self.baud,
                        "serial_open": self.fd is not None,
                        "received_bytes": self.decoder.bytes_received,
                        "valid_packets": self.decoder.valid_packets,
                        "bad_checksums": self.decoder.bad_checksums,
                        "malformed_packets": self.decoder.malformed_packets,
                        "packet_age_s": None
                        if self.last_packet is None
                        else now - self.last_packet,
                        "velocity_age_s": None
                        if self.last_published is None
                        else now - self.last_published,
                        "mounting_confirmed": self.mounting_confirmed,
                        "error": self.error,
                        "last_sample": self.last_sample,
                    },
                    allow_nan=False,
                )
            )
        )


def main():
    rclpy.init()
    node = TeledyneDvl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

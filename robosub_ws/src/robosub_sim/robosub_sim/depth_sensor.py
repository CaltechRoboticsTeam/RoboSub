#!/usr/bin/env python3
"""Convert Gazebo altimeter height to positive-down depth."""
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from std_msgs.msg import Float64

from robosub_control.config import constants
from robosub_control.utils.tuning import fixed_parameter


class DepthSensor(Node):
    def __init__(self):
        super().__init__('depth_sensor')
        self.surface_z = float(
            fixed_parameter(self, 'surface_z', constants.SIM_SURFACE_Z_M)
        )

        self.pub = self.create_publisher(Float64, '/depth', 10)
        self.create_subscription(Altimeter, '/altimeter', self.on_alt, 10)
        self._logged_ref = False
        self.get_logger().info(
            f'depth_sensor up: /altimeter -> /depth '
            f'(surface_z={self.surface_z})')

    def on_alt(self, msg: Altimeter):
        world_z = msg.vertical_reference + msg.vertical_position
        depth = self.surface_z - world_z
        if not self._logged_ref:
            self._logged_ref = True
            self.get_logger().info(
                f'altimeter reference z={msg.vertical_reference:.3f} m '
                f'-> first depth {depth:.3f} m')
        self.pub.publish(Float64(data=depth))


def main():
    rclpy.init()
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():          # a real error; a SIGTERM mid-spin is not
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():          # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ROS I/O only. Geometry and target selection live in pipeline.py."""

from collections import deque
from copy import deepcopy
import json
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from message_filters import Subscriber, TimeSynchronizer
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robosub_vision_msgs.msg import TargetObservation
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage
from tf2_ros import Buffer, TransformException, TransformListener
from zed_msgs.msg import PosTrackStatus

from robosub_vision.calibration import sha256
from robosub_vision.config import load_config
from robosub_vision.depth import roi_pixels, zed_points
from robosub_vision.detector import Detector
from robosub_vision.geometry import rotate_covariance
from robosub_vision.pipeline import Pipeline
from robosub_vision.rectification import camera_fingerprint
from robosub_vision.refraction import RefractiveStereo


def cloud_message(points, header):
    data = np.asarray(points, dtype="<f4").reshape(-1, 3)
    message = PointCloud2()
    message.header = header
    message.height, message.width = 1, len(data)
    message.fields = [
        PointField(name=name, offset=4 * i, datatype=7, count=1)
        for i, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian, message.is_dense = False, False
    message.point_step, message.row_step = 12, 12 * len(data)
    message.data = data.tobytes()
    return message


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("robosub_vision")
        filename = self.declare_parameter("config", "").value
        self.config = load_config(filename)
        config = self.config
        self.bridge = CvBridge()
        self.stereo = None
        water = config["underwater"]
        if water["depth_source"] == "refractive":
            self.stereo = RefractiveStereo(
                water["housing"],
                water["rectification"],
                sha256(config["camera"]["sdk_calibration"]),
            )
        else:
            self.get_logger().warning(
                "UNCORRECTED SDK depth selected; "
                "underwater accuracy is unmeasured"
            )
        self.pipeline = Pipeline(config, self.stereo)
        self.detector = (
            Detector(config["perception"], config["targets"])
            if config["targets"]
            else None
        )
        self.get_logger().info(f"Enabled targets: {len(config['targets'])}")
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf, self)
        self.publishers_by_target = {}
        self.info = {}
        self.info_subscriptions = []
        self.status = deque(maxlen=20)
        self.odometry = deque(maxlen=20)
        self.map_poses = deque(maxlen=20)
        self.last_process = 0
        self.last_frame = time.monotonic()
        self.processed = 0
        self.rejected = 0
        self.last_latency = 0
        root = f"/{config['camera']['name']}/zed_node"
        for side in ("left", "right"):
            for view in ("raw", "rect"):
                key = f"{side}_{view}"
                self.info_subscriptions.append(
                    self.create_subscription(
                        CameraInfo,
                        f"{root}/{side}/color/{view}/camera_info",
                        lambda msg, key=key: self.info.update({key: msg}),
                        qos_profile_sensor_data,
                    )
                )
        self.tracking_sub = self.create_subscription(
            PosTrackStatus,
            root + "/pose/status",
            self.on_status,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            root + "/odom",
            self.odometry.append,
            qos_profile_sensor_data,
        )
        self.map_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            root + "/pose_with_covariance",
            self.map_poses.append,
            qos_profile_sensor_data,
        )
        self.subscribers = [
            Subscriber(
                self, kind, root + topic, qos_profile=qos_profile_sensor_data
            )
            for kind, topic in (
                (Image, "/rgb/color/rect/image"),
                (Image, "/depth/depth_registered"),
                (DisparityImage, "/disparity/map"),
                (Image, "/confidence/confidence_map"),
            )
        ]
        self.sync = TimeSynchronizer(self.subscribers, 3)
        self.sync.registerCallback(self.on_frame)
        self.debug = {
            name: self.create_publisher(PointCloud2, f"/vision/debug/{name}", 1)
            for name in ("zed_roi", "refractive_roi")
        }
        self.diagnostics = self.create_publisher(
            String, "/vision/diagnostics", 1
        )
        self.timer = self.create_timer(5, self.report)

    def on_status(self, message):
        # Vendor status has no Header; gate using recent receipt times.
        self.status.append((self.get_clock().now().nanoseconds / 1e9, message))

    def report(self):
        elapsed = time.monotonic() - self.last_frame
        data = {
            "processed": self.processed,
            "rejected": self.rejected,
            "last_latency_ms": round(self.last_latency, 2),
            "seconds_since_synchronized_frame": round(elapsed, 2),
            "depth_source": self.config["underwater"]["depth_source"],
        }
        self.diagnostics.publish(String(data=json.dumps(data)))
        if elapsed > 5:
            self.get_logger().warning(
                "No synchronized RGB/depth/disparity/"
                "confidence; check wrapper topics and QoS"
            )

    def on_frame(self, rgb, depth_msg, disparity_msg, confidence_msg):
        self.last_frame = time.monotonic()
        if (
            self.last_frame - self.last_process
            < 1 / self.config["perception"]["max_fps"]
        ):
            return
        self.last_process = self.last_frame
        if len(self.info) != 4:
            self.get_logger().warning(
                "Waiting for raw and rectified stereo CameraInfo",
                throttle_duration_sec=5,
            )
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            confidence = self.bridge.imgmsg_to_cv2(confidence_msg, "32FC1")
            disparity = self.bridge.imgmsg_to_cv2(disparity_msg.image, "32FC1")
            # Normalize the signed stereo_msgs baseline/disparity pair.
            if not np.isfinite(disparity_msg.t) or disparity_msg.t == 0:
                raise ValueError(
                    "Disparity baseline must be finite and nonzero"
                )
            disparity = disparity * np.sign(disparity_msg.t)
            k = np.asarray(self.info["left_rect"].p).reshape(3, 4)[:, :3]
            kr = np.asarray(self.info["right_rect"].p).reshape(3, 4)[:, :3]
            if not np.allclose(k, kr, rtol=1e-5, atol=0.01):
                raise ValueError(
                    "Expected matching rectified intrinsics; "
                    "disparity pixel convention needs review"
                )
            if not np.isclose(disparity_msg.f, k[0, 0], rtol=0.001):
                raise ValueError(
                    "Disparity focal length differs from image model"
                )
            if (rgb.height, rgb.width) != depth.shape or any(
                data.shape != depth.shape for data in (confidence, disparity)
            ):
                raise ValueError(
                    "RGB, depth, confidence and disparity sizes differ"
                )
            if any(
                m.header.frame_id != rgb.header.frame_id
                for m in (depth_msg, confidence_msg, disparity_msg)
            ):
                raise ValueError("Image optical frame IDs differ")
            if self.stereo:
                if self.stereo.rect_size != (rgb.width, rgb.height):
                    raise ValueError(
                        "Runtime resolution differs from SDK ray maps"
                    )
                for i, side in enumerate(("left", "right")):
                    fingerprint = camera_fingerprint(
                        self.info[f"{side}_raw"], self.info[f"{side}_rect"]
                    )
                    saved = self.stereo.fingerprints[i]
                    if fingerprint.shape != saved.shape or not np.allclose(
                        fingerprint, saved, atol=1e-4, rtol=1e-6
                    ):
                        raise ValueError(
                            "SDK calibration changed; record new maps"
                        )
            if self.detector is None:
                return
            bgr = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            stamp = Time.from_msg(rgb.header.stamp).nanoseconds / 1e9
            detections = self.detector.detect(bgr)
            observations, errors = self.pipeline.process(
                detections, depth, disparity, confidence, k, stamp
            )
            for error in errors:
                self.rejected += 1
                self.get_logger().warning(error, throttle_duration_sec=3)
            for target, estimate, points in observations:
                self.publish(target, estimate, rgb.header, "camera")
                for frame in self.config["frames"]["output"]:
                    self.publish_in_frame(target, estimate, rgb.header, frame)
                if self.config["perception"]["debug_clouds"]:
                    if self.stereo:
                        self.debug["refractive_roi"].publish(
                            cloud_message(points, rgb.header)
                        )
                    pixels = roi_pixels(
                        max(
                            (
                                d
                                for d in detections
                                if d.class_name == target["detector_class"]
                            ),
                            key=lambda d: d.confidence,
                        ).box,
                        depth.shape,
                        4,
                        1500,
                    )
                    self.debug["zed_roi"].publish(
                        cloud_message(zed_points(pixels, depth, k), rgb.header)
                    )
            self.processed += 1
        except (ValueError, RuntimeError, cv2.error, CvBridgeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)
        finally:
            self.last_latency = (time.monotonic() - self.last_process) * 1000

    def publish_in_frame(self, target, estimate, header, frame):
        stamp = Time.from_msg(header.stamp)
        seconds = stamp.nanoseconds / 1e9
        timeout = self.config["frames"]["tracking_timeout"]
        covariance = None
        if frame in ("odom", "map"):
            statuses = [
                s
                for receipt, s in self.status
                if abs(receipt - seconds) < timeout
            ]
            if (
                not statuses
                or statuses[-1].odometry_status != PosTrackStatus.OK
            ):
                return
            if frame == "map" and statuses[-1].spatial_memory_status not in (
                PosTrackStatus.LOOP_CLOSED,
                PosTrackStatus.KNOWN_MAP,
                PosTrackStatus.MAP_UPDATE,
            ):
                return
            history = self.odometry if frame == "odom" else self.map_poses
            candidates = [
                msg
                for msg in history
                if abs(
                    Time.from_msg(msg.header.stamp).nanoseconds / 1e9 - seconds
                )
                < timeout
            ]
            if not candidates:
                return
            closest = min(
                candidates,
                key=lambda msg: abs(
                    Time.from_msg(msg.header.stamp).nanoseconds / 1e9 - seconds
                ),
            )
            covariance = np.asarray(closest.pose.covariance).reshape(6, 6)
            if not np.isfinite(covariance).all():
                return
        try:
            transform = self.tf.lookup_transform(
                frame, header.frame_id, stamp, timeout=Duration(seconds=0.05)
            )
            tr = transform.transform
            r = Rotation.from_quat(
                [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
            ).as_matrix()
            t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
            output = deepcopy(estimate)
            output.position = r @ estimate.position + t
            output.rotation = (
                r @ estimate.rotation if estimate.rotation is not None else None
            )
            output.axis = (
                r @ estimate.axis if estimate.axis is not None else None
            )
            output.covariance = rotate_covariance(
                estimate.covariance, r, output.rotation is not None
            )
            if covariance is not None:
                link = self.tf.lookup_transform(
                    frame, f"{self.config['camera']['name']}_camera_link", stamp
                )
                origin = link.transform.translation
                lever = output.position - [origin.x, origin.y, origin.z]
                x, y, z = lever
                j = np.eye(6)
                j[:3, 3:] = -np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                extra = j @ covariance @ j.T
                if output.rotation is None:
                    output.covariance[:3, :3] += extra[:3, :3]
                else:
                    output.covariance += extra
            converted_header = deepcopy(header)
            converted_header.frame_id = frame
            self.publish(target, output, converted_header, frame)
        except TransformException:
            self.get_logger().warning(
                f"No {header.frame_id}->{frame} TF at image "
                "time; skipping conversion",
                throttle_duration_sec=5,
            )

    def publish(self, target, estimate, header, suffix):
        key = (target["name"], suffix)
        if key not in self.publishers_by_target:
            root = f"/vision/{target['name']}/{suffix}"
            self.publishers_by_target[key] = (
                self.create_publisher(
                    TargetObservation, root + "/observation", 5
                ),
                self.create_publisher(PointStamped, root + "/position", 5),
                self.create_publisher(
                    PoseWithCovarianceStamped, root + "/pose", 5
                ),
            )
        observation_pub, position_pub, pose_pub = self.publishers_by_target[key]
        msg = TargetObservation()
        msg.header, msg.target, msg.method = (
            header,
            target["name"],
            target["method"],
        )
        msg.depth_source = self.config["underwater"]["depth_source"]
        msg.reference_point = estimate.reference_point
        msg.position.x, msg.position.y, msg.position.z = map(
            float, estimate.position
        )
        msg.orientation_valid = estimate.rotation is not None
        if msg.orientation_valid:
            q = Rotation.from_matrix(estimate.rotation).as_quat()
            msg.orientation.x, msg.orientation.y = float(q[0]), float(q[1])
            msg.orientation.z, msg.orientation.w = float(q[2]), float(q[3])
        msg.covariance = estimate.covariance.ravel().tolist()
        msg.axis_valid = estimate.axis is not None
        if msg.axis_valid:
            msg.axis.x, msg.axis.y, msg.axis.z = map(float, estimate.axis)
        msg.radius_valid = estimate.radius is not None
        if msg.radius_valid:
            msg.radius = estimate.radius
        msg.symmetry, msg.fit_rmse = estimate.symmetry, estimate.rmse
        msg.support = estimate.support
        observation_pub.publish(msg)
        point = PointStamped(header=header, point=msg.position)
        position_pub.publish(point)
        if msg.orientation_valid:
            pose = PoseWithCovarianceStamped()
            pose.header = header
            pose.pose.pose.position, pose.pose.pose.orientation = (
                msg.position,
                msg.orientation,
            )
            pose.pose.covariance = msg.covariance
            pose_pub.publish(pose)


def main():
    rclpy.init()
    node = None
    # tf2 has its own callback group; keep it responsive during inference.
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = PerceptionNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        rclpy.shutdown()

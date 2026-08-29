#!/usr/bin/env python3
from email import message

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf_transformations

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter(
            "bottle_center_plane_z",
            0.04,
        )

        self.bottle_center_plane_z = float(
            self.get_parameter(
                "bottle_center_plane_z"
            ).value
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )

        self.get_logger().info("Color Detector Node Started with TF2 lookup transform")
    def camera_info_callback(
        self,
        message: CameraInfo,
    ) -> None:
        """Store the calibrated camera intrinsics."""

        self.fx = float(message.k[0])
        self.fy = float(message.k[4])
        self.cx = float(message.k[2])
        self.cy = float(message.k[5])

        self.camera_frame = (
            message.header.frame_id
            or "camera_link_optical"
        )

        self.get_logger().info(
            "[CAMERA CALIBRATION] "
            f"frame={self.camera_frame} | "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

        # Intrinsics remain constant, so one message is sufficient.
        self.destroy_subscription(
            self.camera_info_sub
        )
        self.camera_info_sub = None

    def pixel_to_base_point(
        self,
        pixel_u: float,
        pixel_v: float,
    ) -> np.ndarray:
        """Project an image pixel onto the bottle-centre plane."""

        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
            or self.camera_frame is None
        ):
            raise RuntimeError(
                "CameraInfo has not arrived yet"
            )

        transform = self.tf_buffer.lookup_transform(
            "panda_link0",
            self.camera_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )

        camera_origin_base = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=float,
        )

        quaternion_xyzw = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]

        rotation_base_from_camera = (
            tf_transformations.quaternion_matrix(
                quaternion_xyzw
            )[:3, :3]
        )

        # In a ROS optical frame:
        # +X points image-right
        # +Y points image-down
        # +Z points forward from the camera.
        ray_camera = np.array(
            [
                (pixel_u - self.cx) / self.fx,
                (pixel_v - self.cy) / self.fy,
                1.0,
            ],
            dtype=float,
        )

        ray_camera /= np.linalg.norm(ray_camera)

        ray_base = (
            rotation_base_from_camera @ ray_camera
        )

        if abs(ray_base[2]) < 1e-8:
            raise RuntimeError(
                "Camera ray is parallel to the target plane"
            )

        ray_scale = (
            self.bottle_center_plane_z
            - camera_origin_base[2]
        ) / ray_base[2]

        if ray_scale <= 0.0:
            raise RuntimeError(
                "Target plane is behind the camera"
            )

        point_base = (
            camera_origin_base
            + ray_scale * ray_base
        )

        return point_base

    
    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges (HSV)
        color_ranges = {
            "R": [(0, 120, 70), (10, 255, 255)],
            "G": [(55, 200, 200), (60, 255, 255)],
            "B": [(90, 200, 200), (128, 255, 255)]
        }

        for color_id, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            # Noise removal
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 50:  # Increased minimum area threshold
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx_pix, cy_pix = x + w // 2, y + h // 2

                    # Draw bounding box + label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, color_id, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


                    try:
                        point_base = self.pixel_to_base_point(
                            pixel_u=cx_pix,
                            pixel_v=cy_pix,
                        )

                        message_text = (
                            f"{color_id},"
                            f"{point_base[0]:.4f},"
                            f"{point_base[1]:.4f},"
                            f"{point_base[2]:.4f}"
                        )

                        self.coords_pub.publish(
                            String(data=message_text)
                        )

                        self.get_logger().info(
                            f"[DETECTED {color_id}] "
                            f"pixel=({cx_pix}, {cy_pix}) | "
                            f"base=({point_base[0]:.3f}, "
                            f"{point_base[1]:.3f}, "
                            f"{point_base[2]:.3f})"
                        )

                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                        RuntimeError,
                    ) as error:
                        self.get_logger().warn(
                            f"Could not project detected pixel: {error}"
                        )
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error in TF transform: {e}")

        # Show image in window
        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"OpenCV display error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

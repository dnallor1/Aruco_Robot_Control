#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS nodes
from sensor_msgs.msg import Image  # ROS Image message type
from geometry_msgs.msg import Twist  # ROS Twist message type
from cv_bridge import CvBridge  # Bridge between ROS and OpenCV images
import cv2  # OpenCV for image processing
import cv2.aruco as aruco  # ArUco marker detection library


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # CvBridge instance for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscription to image data topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # Ensure this topic is correct
            self.listener_callback,
            10
        )

        # Publisher for robot velocity commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Log the initialization of the node
        self.get_logger().info("CameraNode initialized and subscribing to 'image_raw'.")

        # ArUco marker detection setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

    def generate_twist(self, linear, angular):
        """
        Helper function to create a Twist message for robot control.
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    def process_marker_position(self, marker_center_x, marker_center_y, img_center_x, img_center_y):
        """
        Determines the robot's movement based on marker position relative to image center.
        """
        linear_speed = 0.0
        angular_speed = 0.0

        # Adjust forward/backward motion based on vertical position
        if marker_center_y < img_center_y - 10:
            linear_speed = 0.5  # Move forward
            self.get_logger().info("Marker above center: moving forward.")
        elif marker_center_y > img_center_y + 10:
            linear_speed = -0.5  # Move backward
            self.get_logger().info("Marker below center: moving backward.")

        # Adjust rotational motion based on horizontal position
        if marker_center_x < img_center_x - 10:
            angular_speed = 0.5  # Rotate clockwise
            self.get_logger().info("Marker left of center: rotating clockwise.")
        elif marker_center_x > img_center_x + 10:
            angular_speed = -0.5  # Rotate counterclockwise
            self.get_logger().info("Marker right of center: rotating counterclockwise.")

        return linear_speed, angular_speed

    def listener_callback(self, msg):
        """
        Callback function to process incoming image data and detect ArUco markers.
        """
        try:
            # Convert ROS image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Image dimensions
            img_height, img_width, _ = frame.shape
            img_center_x = img_width / 2
            img_center_y = img_height / 2

            # Convert to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                # Draw detected markers on the frame
                aruco.drawDetectedMarkers(frame, corners, ids)
                self.get_logger().info(f"Detected markers: {ids.flatten().tolist()}")

                # Process each detected marker
                for corner in corners:
                    # Calculate the marker's center
                    center_x, center_y = corner[0].mean(axis=0)

                    # Determine robot movement based on marker position
                    linear, angular = self.process_marker_position(center_x, center_y, img_center_x, img_center_y)

                    # Publish movement command
                    self.cmd_publisher.publish(self.generate_twist(linear, angular))
            else:
                # Stop robot if no markers detected
                self.cmd_publisher.publish(self.generate_twist(0.0, 0.0))
                self.get_logger().info("No ArUco markers detected. Robot stopped.")

            # Display the frame
            cv2.imshow("ArUco Marker Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    """
    Main function to initialize the node and start spinning.
    """
    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

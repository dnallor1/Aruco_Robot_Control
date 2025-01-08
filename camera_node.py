#!/usr/bin/env python3

import rclpy  # ROS 2 Python Client Library
from rclpy.node import Node  # Base class for ROS nodes
from sensor_msgs.msg import Image  # Image message type
from geometry_msgs.msg import Twist  # Twist message for robot control
from cv_bridge import CvBridge  # Convert between ROS Image and OpenCV
import cv2  # OpenCV library
import cv2.aruco as aruco  # ArUco marker detection


class MarkerControlNode(Node):
    def __init__(self):
        super().__init__('marker_control_node')

        # Initialize the CvBridge
        self.image_converter = CvBridge()

        # Subscription to the camera topic
        self.image_subscription = self.create_subscription(
            Image,
            'camera_feed',  # Renamed topic
            self.process_image_callback,
            10
        )

        # Publisher for robot movement commands
        self.movement_publisher = self.create_publisher(Twist, '/robot/movement_commands', 10)

        self.get_logger().info("Marker Control Node initialized and listening to 'camera_feed'.")

        # ArUco dictionary and parameters
        self.marker_dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.detection_parameters = aruco.DetectorParameters_create()

        # Define mapping from marker IDs to robot movement commands
        self.marker_commands = {
            90: self.create_twist(0.4, 0.0),  # Move forward
            30: self.create_twist(0.0, 0.3),  # Rotate left
            0: self.create_twist(-0.4, 0.0), # Move backward
            60: self.create_twist(0.0, -0.3), # Rotate right
        }

    def create_twist(self, linear_x, angular_z):
        """
        Helper function to create a Twist message for robot movement.
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    def process_image_callback(self, image_message):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.image_converter.imgmsg_to_cv2(image_message, "bgr8")

            # Convert to grayscale for marker detection
            grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            marker_corners, marker_ids, _ = aruco.detectMarkers(
                grayscale_frame, self.marker_dictionary, parameters=self.detection_parameters
            )

            # If markers are detected
            if marker_ids is not None:
                # Draw detected markers on the frame
                aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
                detected_ids = marker_ids.flatten()
                self.get_logger().info(f"Detected markers: {detected_ids}")

                # Process each detected marker ID
                for marker_id in detected_ids:
                    if marker_id in self.marker_commands:
                        command = self.marker_commands[marker_id]
                        self.movement_publisher.publish(command)
                        self.get_logger().info(f"Command published for marker ID {marker_id}.")
                    else:
                        self.get_logger().warning(f"No command mapped for marker ID {marker_id}.")

            # Display the frame with detected markers
            cv2.imshow("Marker Detection View", frame)
            cv2.waitKey(1)

        except Exception as error:
            self.get_logger().error(f"Error processing image: {error}")


def main(args=None):
    rclpy.init(args=args)

    # Instantiate the MarkerControlNode
    node = MarkerControlNode()

    try:
        # Keep the node running to handle callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

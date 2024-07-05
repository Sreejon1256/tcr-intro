import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.publisher_screen = self.create_publisher(Point, 'ball_coordinates_screen', 10)
        self.publisher_camera = self.create_publisher(Point, 'ball_coordinates_camera', 10)
        self.image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([30, 150, 50])
        upper_color = np.array([255, 255, 180])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                screen_coords = Point()
                screen_coords.x = float(center[0])
                screen_coords.y = float(center[1])
                screen_coords.z = 0.0
                self.publisher_screen.publish(screen_coords)

                camera_coords = Point()
                camera_coords.x = float((center[0] - frame.shape[1] / 2) / frame.shape[1])
                camera_coords.y = float((center[1] - frame.shape[0] / 2) / frame.shape[0])
                camera_coords.z = float(radius)
                self.publisher_camera.publish(camera_coords)

                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        
        tracked_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_publisher.publish(tracked_image_msg)

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    ball_tracker = BallTracker()
    rclpy.spin(ball_tracker)
    ball_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
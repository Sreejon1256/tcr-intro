#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.turtle1_pose = None
        self.circle_drawn = False
        self.first_turtle_complete = False
        self.second_turtle_spawned = False
        self.second_turtle_publisher_ = None
        self.second_turtle_pose = None
        self.second_turtle_initial_pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()

    def pose_callback(self, msg):
        self.turtle1_pose = msg

    def second_turtle_pose_callback(self, msg):
        self.second_turtle_pose = msg
        self.get_logger().info(f'Second turtle pose: {self.second_turtle_pose}')
        if self.second_turtle_initial_pose is None:
            self.second_turtle_initial_pose = Pose()
            self.second_turtle_initial_pose.x = msg.x
            self.second_turtle_initial_pose.y = msg.y
            self.second_turtle_initial_pose.theta = msg.theta
            self.get_logger().info(f"Initial Pose: {self.second_turtle_initial_pose}")

    def timer_callback(self):
        if not self.circle_drawn:
            msg = Twist()
            msg.linear.x = 1.0  # Move forward
            msg.angular.z = 1.0  # Rotate
            self.publisher_.publish(msg)
            elapsed_time = self.get_clock().now() - self.start_time
            if elapsed_time.nanoseconds > 6.28 * 1e9:  # Roughly one circle
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.circle_drawn = True
                self.get_logger().info('First turtle finished the circle.')
                self.first_turtle_complete = True

        if self.first_turtle_complete and not self.second_turtle_spawned:
            self.spawn_second_turtle()

        if self.second_turtle_spawned:
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = -1.0  # Rotate
            self.second_turtle_publisher_.publish(msg)
            elapsed_time = self.get_clock().now() - self.start_time
            #self.get_logger().info('Publishing to /turtle2/cmd_vel')

            if self.has_second_turtle_returned() and elapsed_time.nanoseconds > 1.8*6.26*1e9:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.second_turtle_publisher_.publish(msg)
                self.get_logger().info('Second turtle returned to initial pose and stopped.')
                self.destroy_node()

    def has_second_turtle_returned(self):
        if self.second_turtle_pose and self.second_turtle_initial_pose:
            distance = math.sqrt((self.second_turtle_pose.x - self.second_turtle_initial_pose.x) ** 2 + 
                                 (self.second_turtle_pose.y - self.second_turtle_initial_pose.y) ** 2)
            self.get_logger().info(f'Distance from initial pose: {distance}')
            return distance < 0.1 # Threshold for considering the turtle as returned
        return False

    def spawn_second_turtle(self):
        self.get_logger().info(f'Spawning second turtle at ({self.turtle1_pose.x}, {self.turtle1_pose.y})')

        spawn_client = self.create_client(Spawn, 'spawn')
        if not spawn_client.service_is_ready():
            self.get_logger().info('Waiting for spawn service...')
            if not spawn_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Spawn service not available')
                return

        request = Spawn.Request()
        request.x = self.turtle1_pose.x
        request.y = self.turtle1_pose.y
        request.theta = self.turtle1_pose.theta
        request.name = 'turtle2'
        future = spawn_client.call_async(request)

        self.get_logger().info(f'Sending spawn request: x={request.x}, y={request.y}, theta={request.theta}, name={request.name}')
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawn response: {response.name}')
            self.setup_second_turtle_publisher()
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')

    def setup_second_turtle_publisher(self):
        self.second_turtle_publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle2/pose', self.second_turtle_pose_callback, 10)
        self.second_turtle_spawned = True
        self.get_logger().info('Second turtle publisher created and ready.')

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


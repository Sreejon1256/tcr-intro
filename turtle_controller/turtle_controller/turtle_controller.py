import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.subscription = self.create_subscription(Point, 'ball_coordinates_screen', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.previous_x = None
        self.previous_y = None

    def listener_callback(self, msg):
        twist = Twist()
        if self.previous_x is None:
            self.previous_x = msg.x
            self.previous_y = msg.y
            return

        if msg.x > self.previous_x:
            twist.linear.x = 2.0
        elif msg.x < self.previous_x:
            twist.linear.x = -2.0
        
        if msg.y > self.previous_y:
            twist.linear.y = 2.0
        elif msg.y < self.previous_y:
            twist.linear.y = -2.0

        self.publisher.publish(twist)
        self.previous_x = msg.x
        self.previous_y = msg.y

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
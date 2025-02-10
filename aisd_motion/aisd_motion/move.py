import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand
from geometry_msgs.msg import Twist

#Defines the class Node which inherits from Node (rclpy.node.Node)
class Move(Node):
    #Initialize the class with the constructor
    def __init__(self):
        #Initialize the node with the name 'move'
        super().__init__('move')
        #Creates a subscriber that listens to the topic 'cmd_hand'
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #Create a publisher that pubilshes a Twist msg in the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    #The function listener_callback is called every time a topic cmd_hand is received
    def listener_callback(self, msg):
        angle = 0.0
        linear = 0.0
        #If xindex is greater than 0.55, it moves the robot right
        if msg.xindex > 0.55:
            self.get_logger().info('right')
            angle = -0.1
        #If xindex is less than 0.45, it moves the robot left
        elif msg.xindex < 0.45:
            self.get_logger().info('left')
            angle = 0.1
        else:
            angle = 0.0
        # If xindex is greater than xpinky, it moves the robot forward
        if msg.xindex > msg.xpinky:
            self.get_logger().info('come')
            linear = 0.5
        #If not, the robot stops
        else:
            self.get_logger().info('stay')
            linear = 0.0
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angle
        #If there is a suscriber, publish the twist message
        if self.vel_publisher.get_subscription_count() > 0:
            self.vel_publisher.publish(twist)
            self.get_logger().info('Publishing a twist message')
        else:
            self.get_logger().info('waiting for subcriber')

def main(args=None):
    rclpy.init(args=args)

    move = Move()

    rclpy.spin(move)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
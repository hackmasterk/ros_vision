# Import necessary ROS2 and other libraries
import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand  # Custom message for hand position
from geometry_msgs.msg import Twist  # Message for velocity commands
from std_srvs.srv import Empty  # Service for clearing the canvas
from turtlesim.srv import TeleportAbsolute  # Service for teleporting the turtle
from time import sleep  # For adding delays

# Define the Move2 class which inherits from Node (rclpy.node.Node)
class Move2(Node):
    # Initialize the class with the constructor
    def __init__(self):
        # Initialize the Node class with the name 'Move2'
        super().__init__('Move2')
        
        # Create a subscriber that listens to the 'cmd_hand' topic for hand position messages
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Create a publisher that publishes Twist messages to the 'cmd_vel' topic for robot movement
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Flag to indicate if the robot is currently performing an action
        self.is_busy = False
        
        # Attempt to create a client for the teleportation service
        try:
            self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
            i = 0
            # Wait for the teleportation service to become available (up to 5 seconds)
            while not self.cli.wait_for_service(timeout_sec=1.0) and i < 5:
                self.get_logger().info('Teleportation Service not available, waiting...')
                i += 1
            self.req = TeleportAbsolute.Request()  # Create a request object for teleportation
        except Exception as e:
            # If the teleportation service is not available, log a message and set variables to None
            self.cli = None
            self.req = None
            self.get_logger().info('No teleportation service available')
        
        # Attempt to create a client for the canvas clearing service
        try:
            self.clear_cli = self.create_client(Empty, '/clear')
            i = 0
            # Wait for the clear service to become available (up to 5 seconds)
            while not self.clear_cli.wait_for_service(timeout_sec=1.0) and i < 5:
                self.get_logger().info('Service not available, waiting for clear...')
                i += 1
        except Exception as e:
            # If the clear service is not available, log a message and set the client to None
            self.clear_cli = None
        
        # Minimum distance between two fingers to be considered as "together"
        self.min_dist = 0.02
    
    # The listener_callback function is called every time a message is received on the 'cmd_hand' topic
    def listener_callback(self, msg):
        self.get_logger().info('Received a message from subscriber')
        
        # Check if the palm of the hand is facing the camera (index finger is to the right of the pinky)
        if msg.xindex > msg.xpinky:
            # Ensure no other action is currently running before starting a new one
            if not self.is_busy:
                # Check if there are subscribers to the 'cmd_vel' topic
                if self.vel_publisher.get_subscription_count() > 0:
                    # Determine the action based on the hand gesture
                    action = self.get_action_by_hand(msg)
                    if action == 0:
                        self.move_forward()  # Move forward
                    elif action == 1:
                        self.draw_circle()  # Draw a circle
                    elif action == 2:
                        self.draw_polygone(2)  # Draw a polygon
                    elif action == 3:
                        self.rotate()  # Rotate
                    elif action == 4 and self.clear_cli is not None:
                        self.clear_canvas()  # Clear the canvas
                    elif action == -1 and self.req is not None:
                        self.teleporte(5.5, 5.5, 1.57)  # Teleport to a specific location
                else:
                    self.get_logger().info('Waiting for subscriber')
            else:
                self.get_logger().info('There is another action going on, please wait')
        # If the back of the hand is facing the camera, stop all actions
        else:
            self.get_logger().info('Stopping')
            self.stop()
    
    # Determine the action based on the relative positions of the fingers
    def get_action_by_hand(self, msg):
        error = 0.02  # Tolerance for finger position comparisons
        
        # If all fingers are close together (thumb, index, middle, ring, and pinky)
        if (abs(msg.xthumb - msg.xindex) < self.min_dist + error and
            abs(msg.xthumb - msg.xmiddle) < self.min_dist + error and
            abs(msg.xthumb - msg.xring) < self.min_dist + error and
            abs(msg.xthumb - msg.xpinky) < self.min_dist + error):
            return -1  # Teleport action
        
        # If the thumb and index finger are close together
        elif abs(msg.xthumb - msg.xindex) < self.min_dist:
            return 1  # Draw a circle
        
        # If the thumb and middle finger are close together
        elif abs(msg.xthumb - msg.xmiddle) < self.min_dist:
            return 2  # Draw a polygon
        
        # If the thumb and ring finger are close together
        elif abs(msg.xthumb - msg.xring) < self.min_dist:
            return 3  # Rotate
        
        # If the thumb and pinky finger are close together
        elif abs(msg.xthumb - msg.xpinky) < self.min_dist:
            return 4  # Clear the canvas
        
        # If the thumb is far from all other fingers
        else:
            return 0  # Move forward
    
    # Teleport the turtle to a specific location
    def teleporte(self, x, y, theta):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.cli.call_async(self.req)  # Send the teleportation request asynchronously
        self.get_logger().info(f'Sending request: x={x}, y={y}, theta={theta}')
    
    # Clear the canvas using the clear service
    def clear_canvas(self):
        clear_request = Empty.Request()
        self.clear_cli.call_async(clear_request)  # Send the clear request asynchronously
        self.get_logger().info('Clearing the canvas...')
    
    # Draw a polygon by moving forward and rotating
    def draw_polygone(self, duration):
        self.is_busy = True  # Mark the robot as busy
        
        self.move_forward()  # Move forward
        sleep(duration)  # Wait while moving forward
        
        self.rotate()  # Rotate
        sleep(duration)  # Wait while rotating
        
        self.is_busy = False  # Mark the robot as not busy
        return
    
    # Draw a circle by moving forward and rotating simultaneously
    def draw_circle(self):
        self.is_busy = True  # Mark the robot as busy
        
        twist = Twist()
        twist.linear.x = 0.5  # Move forward
        twist.angular.z = 0.5  # Rotate
        self.vel_publisher.publish(twist)  # Publish the Twist message
        self.get_logger().info('Drawing a circle')
        
        self.is_busy = False  # Mark the robot as not busy
    
    # Move the robot forward
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.3  # Move forward
        twist.angular.z = 0.0  # No rotation
        self.vel_publisher.publish(twist)  # Publish the Twist message
        self.get_logger().info('Moving forward...')
    
    # Rotate the robot
    def rotate(self):
        twist = Twist()
        twist.linear.x = 0.0  # Stop moving forward
        twist.angular.z = 0.524  # Rotate by 60 degrees (approximately)
        self.vel_publisher.publish(twist)  # Publish the Twist message
        self.get_logger().info('Rotating...')
    
    # Stop all robot movement
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0  # Stop moving forward
        twist.angular.z = 0.0  # Stop rotating
        self.vel_publisher.publish(twist)  # Publish the Twist message
        self.get_logger().info('Stopping any action')

# Define the main function to initialize and run the ROS2 node
def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    
    # Create an instance of the Move2 class
    move2 = Move2()
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(move2)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    move2.destroy_node()
    
    # Shutdown the ROS2 client library
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
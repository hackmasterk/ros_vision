import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # Import the clear service

from turtlesim.srv import TeleportAbsolute

from time import sleep

#Defines the class Node which inherits from Node (rclpy.node.Node)
class Move2(Node):
    #Initialize the class with the constructor
    def __init__(self):
        #Initialize the node with the name 'move'
        super().__init__('Move2')
        #Creates a subscriber that listens to the topic 'cmd_hand'
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #Create a publisher that pubilshes a Twist msg in the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #Is used if we do not want to publish any other twist message
        self.is_busy = False
        
        try:
            self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
            i = 0
            while not self.cli.wait_for_service(timeout_sec=1.0) and i < 5:
                self.get_logger().info('Teleportation Service not available, waiting...')
                i += 1
            self.req = TeleportAbsolute.Request()
        except Exception as e:
            self.cli = None
            self.req = None
            self.get_logger().info('No teleportation service available')
        
        try:
            # Client for clearing the canvas
            self.clear_cli = self.create_client(Empty, '/clear')
            i = 0
            while not self.clear_cli.wait_for_service(timeout_sec=1.0) and i < 5:
                self.get_logger().info('Service not available, waiting for clear...')
                i += 1
        except Exception as e:
            self.clear_cli = None
            
        
        #Minimum distance between two fingers to be considered as they are together
        self.min_dist = 0.02
    
    #The function listener_callback is called every time a topic cmd_hand is received
    def listener_callback(self, msg):
        self.get_logger().info('Received a message from suscriber')
        if msg.xindex > msg.xpinky: #the palm of the hand facing camera
            #Check if the publicher is not already running an action before publishing a new one
            if not self.is_busy :
                #Check if there are some suscribers available
                if self.vel_publisher.get_subscription_count() > 0:
                    action = self.get_action_by_hand(msg)
                    if action == 0:
                        self.move_forward()
                    elif action == 1:
                        self.draw_circle()
                    elif action == 2:
                        self.draw_polygone(2)
                    elif action == 3:
                        self.rotate()
                    elif action == 4 and self.clear_cli is not None:
                        self.clear_canvas()
                    elif action == -1 and self.req is not None:
                        self.teleporte(5.5, 5.5, 1.57)
                else:
                    self.get_logger().info('waiting for subcriber')
            else:
                self.get_logger().info('There is another action going on, wait please')
        #If it is the back of the hand facing the camera
        else:
            self.get_logger().info('stopping')
            self.stop()
    
    
    def get_action_by_hand(self,msg):
        error = 0.02
        #If all the fingers are put together:
        if abs(msg.xthumb - msg.xindex) < self.min_dist+error and abs(msg.xthumb - msg.xmiddle) < self.min_dist+error and abs(msg.xthumb - msg.xring) < self.min_dist+error and abs(msg.xthumb - msg.xpinky) < self.min_dist+error:
            return -1
        #If the thumb and the index are together
        elif abs(msg.xthumb - msg.xindex) < self.min_dist:
            return 1
        #If the thumb and the index are together
        elif abs(msg.xthumb - msg.xmiddle) < self.min_dist:
            return 2
        #If the thumb and the index are together
        elif abs(msg.xthumb - msg.xring) < self.min_dist:
            return 3
        #If the thumb and the index are together
        elif abs(msg.xthumb - msg.xpinky) < self.min_dist:
            return 4
        #If the thumb is fare from all other fingers
        else:
            return 0
            
    
    def teleporte(self, x, y, theta):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Sending request: x={x}, y={y}, theta={theta}')      
    
    def clear_canvas(self):
        clear_request = Empty.Request()
        self.clear_cli.call_async(clear_request)
        self.get_logger().info('Clearing the canvas...')
    
    def draw_polygone(self, duration):
        self.is_busy = True
        
        self.move_forward()
        sleep(duration)  # Sleep while moving forward
        
        self.rotate()
        sleep(duration)  # Sleep while moving forward
        
        #self.move_forward()
        #sleep(duration)  # Sleep while moving forward
        
        #self.rotate()
        #sleep(duration)  # Sleep while moving forward
        
        self.is_busy = False
        return
    
    def draw_circle(self):
        self.is_busy = True
        
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.5 
        self.vel_publisher.publish(twist)
        self.get_logger().info('Drawing a circle')
        #sleep(duration)
        
        self.is_busy = False
    
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.3  
        twist.angular.z = 0.0  # No rotation
        self.vel_publisher.publish(twist)
        self.get_logger().info('Moving forward...')
        
        
    
    def rotate(self):
        twist = Twist()
        twist.linear.x = 0.0  # Stop moving forward
        twist.angular.z = 0.524 # Rotate by 60 degrees
        self.vel_publisher.publish(twist)
        self.get_logger().info('Rotating...')
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0 
        self.vel_publisher.publish(twist)
        self.get_logger().info('Stoping any action')
        

def main(args=None):
    rclpy.init(args=args)

    move2 = Move2()

    rclpy.spin(move2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

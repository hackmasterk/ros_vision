# Import necessary ROS2 and other libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Import MediaPipe for hand gesture recognition
import mediapipe as mp
mp_hands = mp.solutions.hands

# Import custom ROS2 message for hand position
from aisd_msgs.msg import Hand

# Define the Hands class which inherits from Node (rclpy.node.Node)
class Hands(Node):
    # Initialize the class with the constructor
    def __init__(self):
        # Initialize the Node class with the name 'hands'
        super().__init__('hands')
        
        # Create a subscriber that subscribes to the 'video_frames' topic.
        # Whenever an image is received, the `listener_callback` function is called.
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Create a bridge object to convert between OpenCV and ROS2 image formats
        self.br = CvBridge()
        
        # Create a publisher that publishes 'Hand' messages to the 'cmd_hand' topic
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)

    # Define the `listener_callback` function which is called every time a video frame is received
    def listener_callback(self, msg):
        # Log that an image has been received
        self.get_logger().info('I received an image')
        
        # Convert the ROS2 image message to an OpenCV image
        image = self.br.imgmsg_to_cv2(msg)

        # Define constants for the hand landmark indices
        THUMB_TIP = 4
        INDEX_FINGER_TIP = 8
        MIDDLE_FINGER_TIP = 12
        RING_FINGER_TIP = 16        
        PINKY_FINGER_TIP = 20
        
        # Analyze the image for hands using MediaPipe
        with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as myhands:
            
            # To improve performance, mark the image as not writeable to pass by reference
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Process the image to detect hand landmarks
            results = myhands.process(image)
            
            # If hand landmarks are detected, publish their positions
            if results.multi_hand_landmarks:
                # Create a Hand message to store the positions of the fingertips
                msg = Hand()
                msg.xthumb = results.multi_hand_landmarks[0].landmark[THUMB_TIP].x
                msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                msg.xmiddle = results.multi_hand_landmarks[0].landmark[MIDDLE_FINGER_TIP].x
                msg.xring = results.multi_hand_landmarks[0].landmark[RING_FINGER_TIP].x
                msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                
                # If there is at least one subscriber, publish the Hand message
                if self.hand_publisher.get_subscription_count() > 0:
                    self.hand_publisher.publish(msg)
                    self.get_logger().info('Publishing Hand message')
                    # Print the fingertip positions for debugging purposes
                    print("xthumb:", str(msg.xthumb), "xindex:", str(msg.xindex), "xmiddle:", str(msg.xmiddle), "xring:", str(msg.xring), "xpinky:", str(msg.xpinky))
                else:
                    self.get_logger().info('Waiting for subscriber')

# Define the main function to initialize and run the ROS2 node
def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    
    # Create an instance of the Hands class
    hands = Hands()
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(hands)
    
    # Destroy the node explicitly (optional, as it will be done automatically when the garbage collector destroys the node object)
    hands.destroy_node()
    
    # Shutdown the ROS2 client library
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
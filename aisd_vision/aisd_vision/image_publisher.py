# Import necessary ROS2 and other libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# Define the ImagePublisher class which inherits from Node (rclpy.node.Node)
class ImagePublisher(Node):
    # Initialize the class with the constructor
    def __init__(self):
        # Initialize the Node class with the name 'image_publisher'
        super().__init__('image_publisher')
        
        # Create a publisher that will publish Image messages to the 'video_frames' topic
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
        # Set the timer period to 0.2 seconds, which determines how often the timer_callback function is executed
        timer_period = 0.2  # seconds
        
        # Create a timer that will call the timer_callback function every timer_period seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize the OpenCV video capture object to capture images from the default camera (usually the webcam)
        self.cap = cv2.VideoCapture(0)
        
        # Create a bridge object to convert between OpenCV images and ROS2 image messages
        self.br = CvBridge()

    # This function is called every timer_period seconds while the Node is running
    def timer_callback(self):
        # Capture an image or frame from the camera. 
        # `ret` is a boolean indicating whether the image capture was successful.
        # `frame` contains the captured image.
        ret, frame = self.cap.read()
        
        # If the image was captured successfully
        if ret == True:
            # Convert the OpenCV image to a ROS2 image message and publish it to the 'video_frames' topic
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            
            # Log a message indicating that the image was published successfully
            self.get_logger().info('Image published successfully')
        else:
            # Log a message if the video capture failed
            self.get_logger().info('Cannot capture video')

# Define the main function to initialize and run the ROS2 node
def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    
    # Create an instance of the ImagePublisher class
    image_publisher = ImagePublisher()
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(image_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    
    # Shutdown the ROS2 client library
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
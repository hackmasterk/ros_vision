import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


#Defines the class ImagePublisher which inherits from Node (rclpy.node.Node)
class ImagePublisher(Node):
    #Initialize the class with the constructor
    def __init__(self):
        #Initialize the Node class whith the name image_plublisher
        super().__init__('image_publisher')
        #Creates publisher that will publssh Image with the topic 'Video_frames'
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        #The time set how long the publisher will excecuter the timber_callback function
        timer_period = 0.2  # seconds
        #Create a timer that will call the timer_callback function
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Initialize the OpenCV video capture object to take pictures of the camera
        self.cap = cv2.VideoCapture(0)
        #Creates a bridghe object to convert between OpenCV and ROS2 images
        self.br = CvBridge()

    #This function is called every timer_period if the Node is running
    def timer_callback(self):
        #Capture an image or frame from OpenCV. ret contains True/False if the image capture was successfulll
        ret, frame = self.cap.read()
        #If the image is captured
        if ret == True:
            #Convert the image to Ros2 format and publish it to the topic
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Image published successfully')
        else:
            self.get_logger().info('Cannot capture video')



def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

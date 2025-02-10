import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import mediapipe as mp
mp_hands = mp.solutions.hands

from aisd_msgs.msg import Hand

#Defines the class Hands which inherits from Node (rclpy.node.Node)
class Hands(Node):
    #Initialize the class with the constructor
    def __init__(self):
        #Initialize the Node class whith the name hands
        super().__init__('hands')
        #Creates a subscriber which suscribes to the topic 'video_frames' and everytime
        #an image is received, call teh function listener_callback
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #Creates a bridghe object to convert between OpenCV and ROS2 images
        self.br = CvBridge()
        #Creates a publisher that publish 'Hand' messages in the topic cmd_hand
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)

    #Defines the function listener_callback which is called every time a video_frame is received
    def listener_callback(self, msg):
        self.get_logger().info('I received an image')
        #Reconvert the image from ROS2 format to OpenCV format
        image = self.br.imgmsg_to_cv2(msg)

        THUMB_TIP = 4
        INDEX_FINGER_TIP = 8
        MIDDLE_FINGER_TIP = 12
        RING_FINGER_TIP = 16        
        PINKY_FINGER_TIP = 20
        # Analyse the image for hands with mediapipe
        with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as myhands:
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = myhands.process(image)
            if results.multi_hand_landmarks:
                #publish the hand position in terms of index finger and pinky
                msg = Hand()
                msg.xthumb = results.multi_hand_landmarks[0].landmark[THUMB_TIP].x
                msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                msg.xmiddle= results.multi_hand_landmarks[0].landmark[MIDDLE_FINGER_TIP].x
                msg.xring = results.multi_hand_landmarks[0].landmark[RING_FINGER_TIP].x
                msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                #If there is a suscriber, we publish the msg which contains the x positions of index and pinky
                if self.hand_publisher.get_subscription_count() > 0:
                    self.hand_publisher.publish(msg)
                    self.get_logger().info('Publishing Hand message')
                    print("xthumb:", str(msg.xthumb), "xindex:", str(msg.xindex), "xmiddle:", str(msg.xmiddle), "xring:", str(msg.xring), "xpinky:", str(msg.xpinky))
                else:
                    self.get_logger().info('waiting for subcriber')



def main(args=None):
    rclpy.init(args=args)

    hands = Hands()

    rclpy.spin(hands)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hands.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

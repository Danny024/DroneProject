'''
Author : Daniel Eneh
@danieleneh024@gmail.com

This is a code for Video frame publishing.
It helps to convert videos to ROS image streams

'''
#!/usr/bin/env python3
#libraries used
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

class VideoImagePublisher(Node):

    def __init__(self, capture):
        #The Node is defined as video_image_pub
        super().__init__('video_image_pub')

        #Creating an Image Publisher
        self.publisher_ = self.create_publisher(Image, '/camera',1)

        #timer callback
        timer_period = 0.1
        self.timer = self.create_publisher(timer_period,self.image_callback)

        #Initialize the image counter
        self.i = 0

        #store the video capture object
        self.capture = capture
    
    #Callback to Capture and Publish images
    def image_callback(self):
        # Check if the video capture is open
        if self.capture.isOpened():
            #Take frames from video capture
            ret, frame = self.capture.read()

            #resize the frame to (640, 480)
            frame = cv2.resize(frame, (640, 480))

            # Image message to be created
            msg = Image()

            #Set the message header timestamp and frame ID
            msg.header.stamp = Node.get_clock(self).now().to_msg()
            msg.header.frame_id = "base"

            #Set the width and height of the image
            msg.width = frame.shape[1]
            msg.width = frame.shape[0]

            #set the encoding format of the image
            msg.encoding = "bgr8"

            #endianness of the image data
            msg.is_bigendian = False

            #Set the step size (row length in bytes)
            msg.step = np.shape(frame)[2] * np.shape(frame)[1]

            #convert frames to bytes
            msg.data = np.array(frame).tobytes()

            #publish the image message
            self.publisher_.publish(msg)

            #Log and print the number of images published
            self.get_logger().info('%d Images Published' % self.i)

        #Increase the image counter
        self.i += 1

#Main function to initialize and run node
def main(args=None):
    #Get path to video using command line
    video_path = sys.argv[1]

    # Open a connection to the specified video file
    capture = cv2.VideoCapture(video_path)

    #Set the buffer size for the  video capture
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    #Initialize the rclpy library
    rclpy.init(args=args)

    #Initialize the CamImagePublisher Object
    cam_img_publisher = None

    #Check if the Video Capture is opened successfully
    if not capture.isOpened():
        #error log
        print("Error: could not open video stream.")

        #Stop Node and Shutdown rclpy
        cam_img_publisher.destroy_node()
        rclpy.shutdown()

        #release the video capture object
        capture.release()
    
    else:
        #create the CamemagePublisher object and start the node
        cam_img_publisher = VideoImagePublisher(capture)

        #spin the node to keep it running 
        rclpy.spin(cam_img_publisher)

if __name__ == '__main__':
    main()




    


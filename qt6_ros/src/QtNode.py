#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String, Int16
import tf

from PyQt6.QtCore import QThread 
from PyQt6.QtGui import QImage
import cv2
import numpy as np

class QtNode(QThread):
    """
    QtNode class serves as a bridge between ROS and a PyQt6 application, 
    running in a separate thread. It listens to multiple ROS topics 
    (e.g., IMU data, compressed camera images, and status messages), 
    processes the incoming data, and communicates with the PyQt6 interface.

    Methods:
        - string_callback(self, msg): Processes String data and sends it to the bridge.
        - compressed_image_callback(msg): Converts compressed image messages to OpenCV format, then updates the QML image provider.
        - run(): Initializes the ROS node and subscribers, then spins until shutdown.
        - stop(): Cleanly stops the thread and shuts down the ROS node.
    """

    def __init__(self, bridge, image_provider):
        QThread.__init__(self)
        self.bridge = bridge
        self.image_provider = image_provider
        self._running = True
    
    def compressed_image_callback(self, msg):
        """
        Callback function for handling incoming compressed image data. Converts the 
        compressed image to an OpenCV format and updates the QML image provider with the new image.

        Input:
          - msg (CompressedImage): ROS CompressedImage message containing camera frame data.

        Output: Converts the image and updates the QML image provider if the image is valid.
        """

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is not None and cv_image.size > 0:
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()

            self.image_provider.updateImage(q_image)
        else:
            print("Failed to decode image or empty image received")

        
        
    
    def string_callback(self, msg):
        self.bridge.getRosData(str(msg))

    def run(self):
        """
        Main execution function of the QtNode thread. Initializes the ROS node and 
        subscribes to multiple ROS topics. The node keeps running until it is 
        manually stopped or ROS shuts down.

        Subscribed Topics:
          - /camera/color/image_rect_color/compressed: Compressed camera frames.
          - chatter: string type message sent by the test_node created
        
        Output: Runs the ROS event loop until manually stopped.
        """

        rospy.init_node("listener", anonymous=True, disable_signals=True)
        rospy.Subscriber("chatter", String, self.string_callback)
        rospy.Subscriber("/camera/color/image_rect_color/compressed", CompressedImage, self.compressed_image_callback)

        while not rospy.is_shutdown() and self._running:
            rospy.spin()

    def stop(self):
        # Cleanly stop the thread
        self._running = False
        rospy.signal_shutdown("Shutting down ROS node...")
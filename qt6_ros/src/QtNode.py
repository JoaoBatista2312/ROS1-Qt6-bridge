#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

import cv2
import numpy as np

class QtNode(QThread):
    """
    The QtNode class integrates ROS with a PySide6 application by running as a separate 
    QThread, allowing it to subscribe to ROS topics and process incoming data 
    independently of the main GUI thread. It listens to specific ROS topics such as 
    camera image data and status messages, processes the data (e.g., decoding images), 
    and emits signals to update the PySide6 GUI.

    Attributes:
        - updateImage (Signal): Signal emitted with a QImage to update the GUI with new camera images.
        - updateMessage (Signal): Signal emitted with a string message to update the GUI with text data.

    Methods:
        - compressed_image_callback(msg): Receives and processes compressed camera image messages, converting them 
          to QImage format for GUI display.
        - string_callback(msg): Receives and processes text messages, emitting them as string data for the GUI.
        - run(): Initializes and runs the ROS node, listening for incoming messages until the node is shut down.
        - stop(): Gracefully stops the thread and shuts down the ROS node.
    """
     
    updateImage = Signal(QImage)   
    updateMessage = Signal(str) 

    def __init__(self):
        super(QtNode, self).__init__()

        rospy.init_node("listener", anonymous=True, disable_signals=True)
        rospy.Subscriber("chatter", String, self.string_callback)
        rospy.Subscriber("/camera/color/image_rect_color/compressed", CompressedImage, self.compressed_image_callback)
        self._running = True
    

    def compressed_image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is not None and cv_image.size > 0:
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()

            self.updateImage.emit(q_image)
        else:
            print("Failed to decode image or empty image received")

        
    def string_callback(self, msg):
        self.updateMessage.emit(str(msg))


    def run(self):
        while not rospy.is_shutdown() and self._running:
            rospy.spin()


    def stop(self):
        self._running = False
        rospy.signal_shutdown("Shutting down ROS node...")
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Node:
    def __init__(self):
        pass
    
    def ros_callback(self, msg, bridge):
        """Callback function to handle ROS messages."""
        bridge.setRosData(msg.data)

    def start_ros_node(self, bridge):
        """
        Starts the ROS subscriber and processes callbacks. This method should be called
        after `rospy.init_node()` has already been initialized in the main thread.
    
        Args:
            bridge (RosQmlBridge): The QML-ROS bridge instance to update QML data.
        """
        
        rospy.Subscriber("/chatter", String, self.ros_callback, bridge)
        
        

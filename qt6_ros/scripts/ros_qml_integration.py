#!/usr/bin/env python3

import sys
import os
import rospy
import rospkg
from std_msgs.msg import String
from PyQt6.QtCore import QObject, pyqtSignal, pyqtProperty, QTimer
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQml import QQmlApplicationEngine


# Add the src directory to the Python path
rospack = rospkg.RosPack()
package_path = rospack.get_path('qt_ros')  # Get the path to your ROS package
sys.path.append(os.path.join(package_path, 'src'))

from qml_retrieval_node import Node  

class RosQmlBridge(QObject):
    """
    A class that acts as a bridge between ROS and a QML-based GUIs

    This class is designed to facilitate communication between ROS nodes and a QML interface by exposing ROS data as properties that
    can be accessed and updated in the QML UI. It uses PyQt6 to integrate ROS data into Qt Quick applications.

    Attributes:
        rosDataChanged (pyqtSignal): Signal emitted when the ROS data is updated. This signal notifies the QML interface
                                     to refresh the displayed data.

    Methods:
        rosData (self) -> str:
            Getter method for the `rosData` property. This property holds the most recent data received from the ROS topic.
        
        setRosData(self, data: str) -> None:
            Setter method for the `rosData` property. Updates the internal `_ros_data` attribute with new data and emits the
            `rosDataChanged` signal if the data has changed.
    """

    rosDataChanged = pyqtSignal()

    def __init__(self):
        super(RosQmlBridge, self).__init__()
        self._ros_data = "" 

    @pyqtProperty(str, notify=rosDataChanged)
    def rosData(self):
        return self._ros_data

    def setRosData(self, data):
        if data != self._ros_data:
            self._ros_data = data
            self.rosDataChanged.emit()


def main():
    # Initialize ROS in the main thread
    rospy.init_node('qt_ros_subscriber_py_node', anonymous=True)

    # Initialize Qt
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # Initialize the ROS-QML bridge
    bridge = RosQmlBridge()

    # Initialize the ROS node (without calling init_node again)
    ros_node = Node()
    ros_node.start_ros_node(bridge)  # Start the subscriber and callback handling

    # Load QML file
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('qt_ros')
    qml_file = os.path.join(package_path, "qml/main.qml")

    engine.rootContext().setContextProperty("rosBridge", bridge)
    engine.load(qml_file)

    if not engine.rootObjects():
        sys.exit(-1)

    # Timer to periodically check ROS messages
    timer = QTimer()

    def check_ros():
        rospy.rostime.wallsleep(0.1)  # Sleep for 100ms to allow ROS callbacks to be processed

    timer.timeout.connect(check_ros)
    timer.start(100)

    # Start the Qt event loop
    sys.exit(app.exec())  # Note: exec_() is renamed to exec() in PyQt6


if __name__ == '__main__':
    main()

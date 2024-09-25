#!/usr/bin/env python3

"""
Made by: Joao Batista -> https://github.com/JoaoBatista2312

Description:
This program integrates a ROS node with a QML-based GUI, allowing data to be displayed 
and updated in real-time. It utilizes ROS for handling sensor data and messages, 
and Qt for building the graphical interface.

License:
This code is licensed under the GNU General Public License v3.0 (GPLv3). 
You may use, distribute, and modify this code under the terms of the GPLv3 license. 
For more details, see https://www.gnu.org/licenses/gpl-3.0.en.html.
"""


import sys
import os
import threading  
import rospy
import rospkg
from PyQt6.QtCore import QObject, pyqtSignal, pyqtProperty
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQml import QQmlApplicationEngine


# Add the src directory to the Python path to allow importing custom modules
rospack = rospkg.RosPack()
package_path = rospack.get_path('qt6_ros')  
sys.path.append(os.path.join(package_path, 'src'))  

from qml_retrieval_node import Node 

class RosQmlBridge(QObject):
    """
    A class that acts as a bridge between ROS and QML-based GUIs.
    This bridge allows data to be transferred between ROS topics and the QML interface.
    Properties are exposed as QML-friendly variables that can be accessed in the GUI.
    """

    # pyqtSignal is emitted whenever the ROS data changes, notifying QML components to update
    rosDataChanged = pyqtSignal()

    def __init__(self):
        super(RosQmlBridge, self).__init__()
        self._ros_data = ""  

    @pyqtProperty(str, notify=rosDataChanged)
    def rosData(self):
        """
        Getter function for the ROS data. Exposes the internal _ros_data variable to QML.
        Whenever this data changes, the rosDataChanged signal is emitted to notify QML.

        Returns:
            str: The current string data from ROS, to be used in the QML interface.
        """

        return self._ros_data

    def setRosData(self, data):
        """
        Setter function to update the internal ROS data. 
        If the new data is different from the current one, it emits a signal to notify QML.

        Args:
            data (str): New ROS data string to be set in the _ros_data variable.
        
        Side effects:
            Updates the internal _ros_data and emits rosDataChanged signal to QML.
        """

        if data != self._ros_data:  # Only update if the data has changed
            self._ros_data = data
            self.rosDataChanged.emit()  # Emit signal to inform QML of the change


def ros_spin_thread():
    """
    This function runs the ROS spin loop in a separate thread.
    ROS's spin loop is needed to continuously process incoming ROS messages and callbacks.
    
    This function allows the Qt application to run in parallel without blocking the ROS node.
    
    Input: None
    Output: Continuously processes ROS callbacks until shutdown.
    """
    rospy.spin()  


# Handle Qt application quit and clean shutdown
def handle_quit(ros_spin):
    """
    Handles the clean shutdown of the ROS node when the Qt application is about to quit.
    This function ensures that ROS is properly shutdown and the ROS spin thread is joined.

    Args:
        ros_spin (threading.Thread): The thread running the ROS spin loop.
    
    Side effects:
        Signals ROS to shut down, waits for the ROS thread to terminate, and cleans up.
    """

    rospy.signal_shutdown('Qt application closed')  
    ros_spin.join()  
    
    print("ROS node shut down cleanly.")  


def main():
    """
    Main entry point for the ROS-Qt application. 
    Initializes both ROS and the Qt framework, sets up the QML interface, 
    and starts the ROS node in a separate thread to keep both ROS and Qt responsive.

    Steps:
    1. Initialize the ROS node.
    2. Create and configure the Qt application.
    3. Set up the bridge between ROS and QML for data communication.
    4. Load the QML UI file and integrate the RosQmlBridge into the QML context.
    5. Start the ROS spin loop in a separate thread.
    6. Manage the application's lifecycle and handle clean shutdown.
    
    Input: None
    Output: Runs the event loops for both ROS and Qt. Application exits on completion.
    """

    # Initialize ROS with an anonymous node
    rospy.init_node('qt_ros_subscriber_py_node', anonymous=True)
    
    # Initialize the Qt application and QML engine
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # Create an instance of the RosQmlBridge to communicate between ROS and QML
    bridge = RosQmlBridge()

    # Initialize the custom ROS node 
    ros_node = Node()
    ros_node.start_ros_node(bridge)  # Start the ROS node and connect to the bridge

    # Load the QML file (the user interface) from the ROS package path
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('qt6_ros') 
    qml_file = os.path.join(package_path, "qml/main.qml")  
    
    # Expose the RosQmlBridge object to the QML context so QML components can access it
    engine.rootContext().setContextProperty("rosBridge", bridge)
    engine.load(qml_file)  

    # Exit the application if the QML file fails to load (empty or invalid QML)
    if not engine.rootObjects():
        sys.exit(-1)

    # Start the ROS spin loop in a separate thread to keep ROS running while Qt is active
    ros_spin = threading.Thread(target=ros_spin_thread)
    ros_spin.start()  

    # Connect the Qt application quit signal to the ROS shutdown handler
    app.aboutToQuit.connect(lambda: handle_quit(ros_spin))

    # Start the Qt event loop, which will continue running until the user closes the application
    sys.exit(app.exec())  


if __name__ == '__main__':
    main()  
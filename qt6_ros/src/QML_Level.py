#!/usr/bin/env python3

from PySide6.QtCore import Signal, Slot, QObject, Property, QUrl
from PySide6.QtGui import QImage, QGuiApplication, QColor
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickImageProvider

import sys
import os

import rospkg
from QtNode import QtNode  

class ImageProvider(QQuickImageProvider):
    """
    The ImageProvider class supplies images to the QML UI. It stores an image that can be updated
    and tracks how many times it has been updated, emitting a signal whenever the update count changes.
    
    Attributes:
        - imageCounterSignal: Signal emitted when the internal image update count changes.
    
    Methods:
        - updateImage(new_image): Updates the current image with `new_image`, increments the update count,
                                  and emits the update signal.
        - requestImage(p_str, size): Returns the stored image if available or a default red image if not.
        - imageUpdateCount (property): Property for accessing the update count, notifying QML of changes.
    """

    imageCounterSignal = Signal()

    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image)  
        self.image = QImage() 
        self._update_count = 0 
    
    @Slot(QImage)
    def updateImage(self, new_image):
        """
        Updates the stored image with `new_image` and emits a signal if the image is a QImage.
        """
        if isinstance(new_image, QImage):
            self.image = new_image  
            self._update_count += 1  
            self.imageCounterSignal.emit()  
        else:
            print("Received image is not of type QImage.")  

    def requestImage(self, id, size, requestedSize):
        if self.image.isNull():
            img = QImage(300, 300, QImage.Format.Format_RGBA8888)
            img.fill(QColor(255, 0, 0)) 
            print("Image is null, returning red image")
            return img

        return self.image 

    @Property(int, notify=imageCounterSignal)  
    def imageUpdateCount(self):
        return self._update_count  


class MyApplicationConnection(QObject):
    """
    The MyApplicationConnection class facilitates data exchange between the ROS framework and QML UI.
    It stores ROS data and notifies the QML interface whenever new data is received.
    
    Attributes:
        - rosDataChanged: Signal emitted when the internal ROS data changes.
    
    Methods:
        - getRosData(data): Updates internal ROS data if it changes and emits rosDataChanged.
        - rosData (property): Provides QML with the current ROS data and notifies changes.
    """

    rosDataChanged = Signal()

    def __init__(self):
        super(MyApplicationConnection, self).__init__()
        self._ros_data = "Hello World"  

    @Slot(str)
    def getRosData(self, data):
        if data != self._ros_data:
            self._ros_data = data
            self.rosDataChanged.emit()  
    @Property(str, notify=rosDataChanged)
    def rosData(self):
        return self._ros_data


if __name__ == "__main__":
    # Initialize the Qt application
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # Set up the image provider and add it to the QML engine
    image_provider = ImageProvider()
    engine.addImageProvider("myImageProvider", image_provider)  # Make provider available in QML
    engine.rootContext().setContextProperty("imageProvider", image_provider)

    # Set up ROS-QML connection and make it available to QML
    connection = MyApplicationConnection()
    engine.rootContext().setContextProperty("appConnection", connection)

    # Load the QML user interface file from the ROS package directory
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('qt6_ros')
    qml_file = os.path.join(package_path, "qml/main.qml")
    engine.load(QUrl.fromLocalFile(qml_file))

    if not engine.rootObjects():
        sys.exit(-1)  

    ros_node = QtNode()

    # Connect ROS node signals to appropriate slots
    ros_node.updateImage.connect(image_provider.updateImage)  
    ros_node.updateMessage.connect(connection.getRosData)  
    ros_node.start()

    def cleanup():
        ros_node.stop()
        ros_node.wait()

    app.aboutToQuit.connect(cleanup)  
    sys.exit(app.exec())  

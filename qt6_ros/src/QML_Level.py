#!/usr/bin/env python3

import sys
from PyQt6.QtCore import pyqtSignal, QObject, pyqtProperty, QUrl
from PyQt6.QtGui import QImage, QGuiApplication, QColor
from PyQt6.QtQml import QQmlApplicationEngine
from PyQt6.QtQuick import QQuickImageProvider
import os
import rospkg

from QtNode import QtNode


class ImageProvider(QQuickImageProvider):
    """
    ImageProvider class responsible for handling and providing images to the QML UI. 
    It stores an internal image and can update the image based on new data. The class
    also tracks the number of image updates and emits a signal when this count changes.

    Methods:
        - updateImage(new_image): Updates the stored image with a new QImage and emits the update signal.
        - requestImage(p_str, size): Returns the current image or a default red image if no image is available.
        - imageUpdateCount (property): Returns the number of image updates and notifies changes.
    """

    imageUpdateCountChanged = pyqtSignal()

    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image) 
        self.image = QImage()
        self._update_count = 0  

    def updateImage(self, new_image):      
        if isinstance(new_image, QImage):
            self.image = new_image  
            self._update_count += 1 
            print("Image updated in provider, update count:", self._update_count)
            
            self.imageUpdateCountChanged.emit()
        else:
            print("Received image is not of type QImage.") 

    def requestImage(self, p_str, size):
        if self.image.isNull():
            img = QImage(300, 300, QImage.Format.Format_RGBA8888)
            img.fill(QColor(255, 0, 0))
            print("Image is null, returning red image")
            return img, img.size()

        print("Image is not null, returning updated image")
        return self.image, self.image.size()

    @pyqtProperty(int, notify=imageUpdateCountChanged)  
    def imageUpdateCount(self):
        return self._update_count  


class MyApplicationConnection(QObject):
    """
    MyApplicationConnection class manages the communication between ROS data and the QML UI.
    It stores the current ROS data and emits a signal when the data changes to notify QML.

    Methods:
      - getRosData(data): Updates the internal ROS data if it has changed and emits the rosDataChanged signal.
      - rosData (property): Exposes the current ROS data to QML and notifies changes.
    """

    rosDataChanged = pyqtSignal()

    def __init__(self):
        super(MyApplicationConnection, self).__init__()
        self._ros_data = ""

    @pyqtProperty(str, notify=rosDataChanged)
    def rosData(self):
        return self._ros_data

    def getRosData(self, data):
        if data != self._ros_data:
            self._ros_data = data
            self.rosDataChanged.emit()



if __name__ == "__main__":
    # Initialize Qt
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # Instantiate the image provider
    image_provider = ImageProvider()
    engine.addImageProvider("myImageProvider", image_provider)
    engine.rootContext().setContextProperty("imageProvider", image_provider)

    # Instantiate connection class
    connection = MyApplicationConnection()

    # Load QML
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('qt6_ros')
    qml_file = os.path.join(package_path, "qml/main.qml")
    engine.rootContext().setContextProperty("appConnection", connection)
    engine.load(QUrl.fromLocalFile(qml_file))

    if not engine.rootObjects():
        sys.exit(-1)

    # Start ROS node
    ros_node = QtNode(connection, image_provider)
    ros_node.start()

    def cleanup():
        ros_node.stop()
        ros_node.wait()

    app.aboutToQuit.connect(cleanup)

    sys.exit(app.exec())

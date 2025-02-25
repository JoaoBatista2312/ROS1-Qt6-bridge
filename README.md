# ROS1 Qt6 bridge

## Description

The `qt6_ros` package provides a seamless integration between the ROS and the Qt framework, enabling the development of robust GUIs for robotic applications. This package is designed to facilitate real-time data visualization and interaction with ROS nodes. This is still under development, so bugs may appear.

The newest version (2024/10/25) uses QQuickImageProvider class in order to display the images on a seperate thread. It also uses the QtNode in a seperate QThread. It was further updated from PyQt6 to PySide6 due to PySide being officially the suported Python Qt API (it also has good documentation).

### Features

- **ROS Integration**: Easily connect your Qt applications with ROS topics and services.
- **Data Visualization**: Monitor and visualize various data types from ROS nodes in an intuitive GUI.
- **Cross-Platform Compatibility**: Built on Qt, ensuring compatibility across different operating systems.
- **Modular Design**: The architecture allows for easy expansion and modification to suit specific application requirements.

## Prerequisites

Before using the `qt6_ros` package, ensure that you have the following software installed:

1. **ROS**:
   - Make sure you have a compatible version of ROS installed. This package has been tested with ROS Noetic.
   - Follow the [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) for your operating system.

2. **Python**:
   - Ensure that Python 3 is installed on your system, as this package is written in Python.
   - You can download Python from [python.org](https://www.python.org/downloads/).

3. **PyQt6**:
   - Install PySyde6, which is required for the graphical user interface. You can install it using pip:
     ```
     pip install Pysyde6
     ```

## Installation

To install the `qt6_ros` package, clone this repository into your ROS workspace and build it using the following commands:

```
cd ~/catkin_ws/src
git clone https://github.com/JoaoBatista2312/ROS1-Qt6-bridge.git
catkin_make
```

## Usage
To run the launch file, ensure your ROS environment is properly set up and compiled, and execute the following command:
```
roslaunch qt6_ros GUI_interface.launch
```
This will launch the Qt GUI, connecting it to the specified test '/chatter' topic for data visualization and the GUI Node.

In order to launch the GUI and the QtNode it is only needed to run the QML_Level.py file.

## License

This package is licensed under the GNU General Public License v3 (GPLv3). You are free to modify and redistribute this package under the same license terms. For more details, please refer to the GPLv3 license.


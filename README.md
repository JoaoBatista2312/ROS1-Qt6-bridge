# ROS1-Qt6-bridge

## Description

The `qt6_ros` package provides a seamless integration between the Robot Operating System (ROS) and the Qt framework, enabling the development of robust graphical user interfaces (GUIs) for robotic applications. This package is designed to facilitate real-time data visualization and interaction with ROS nodes, particularly for applications involving camera feeds and data streams.

### Features

- **ROS Integration**: Easily connect your Qt applications with ROS topics and services.
- **Data Visualization**: Monitor and visualize various data types from ROS nodes in an intuitive GUI.
- **Cross-Platform Compatibility**: Built on Qt, ensuring compatibility across different operating systems.
- **Modular Design**: The architecture allows for easy expansion and modification to suit specific application requirements.

## Installation

To install the `qt6_ros` package, clone this repository into your ROS workspace and build it using the following commands:

```
cd ~/catkin_ws/src
git clone https://github.com/JoaoBatista2312/ROS1-Qt6-bridge.git
catkin_make
```

## Usage
To run the launch file, ensure your ROS environment is properly set up and execute the following command:
```
roslaunch qt6_ros GUI_interface.launch
```
This will launch the Qt application, connecting it to the specified test topic '/chatter' for data visualization and the GUI Node.

## License

This package is licensed under the GNU General Public License v3 (GPLv3). You are free to modify and redistribute this package under the same license terms. For more details, please refer to the GPLv3 license.


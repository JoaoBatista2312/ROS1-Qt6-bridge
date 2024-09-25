# ROS1 Qt6 bridge

## Description

The `qt6_ros` package provides a seamless integration between the Robot Operating System (ROS) and the Qt framework, enabling the development of robust graphical user interfaces (GUIs) for robotic applications. This package is designed to facilitate real-time data visualization and interaction with ROS nodes.

### Features

- **ROS Integration**: Easily connect your Qt applications with ROS topics and services.
- **Data Visualization**: Monitor and visualize various data types from ROS nodes in an intuitive GUI.
- **Cross-Platform Compatibility**: Built on Qt, ensuring compatibility across different operating systems.
- **Modular Design**: The architecture allows for easy expansion and modification to suit specific application requirements.

## Prerequisites

Before using the `qt6_ros` package, ensure that you have the following software installed:

1. **Robot Operating System (ROS)**:
   - Make sure you have a compatible version of ROS installed. This package has been tested with ROS Noetic and Melodic.
   - Follow the [ROS installation guide](http://wiki.ros.org/ROS/Installation) for your operating system.

2. **Qt Framework**:
   - Install Qt version 5.12 or higher. You can download it from the [Qt official website](https://www.qt.io/download).

3. **Python**:
   - Ensure that Python 3 is installed on your system, as this package is written in Python.
   - You can download Python from [python.org](https://www.python.org/downloads/).

4. **Python Packages**:
   - Install the required Python packages. You can do this using pip:
     ```bash
     pip install -r requirements.txt
     ```
   - Make sure to create a `requirements.txt` file that includes any additional dependencies your package might need.

5. **catkin Build System**:
   - Make sure you have the `catkin` build system installed. If you have ROS installed, it should already be available.
   - Refer to the [catkin documentation](http://wiki.ros.org/catkin) for details on usage.

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


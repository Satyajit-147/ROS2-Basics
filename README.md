# ROS2-Basics - Week 1

This README documents my initial learning journey with ROS 2 — covering key concepts, essential commands, workspace structure, challenges encountered, and the implementation of basic publisher and subscriber nodes.

## Learnings

This was my first hands-on experience with ROS 2 (Robot Operating System 2), and it gave me a strong foundational understanding of how ROS-based systems are structured and how different components interact. Here's a breakdown of the key concepts I learned and understood during the task:

### ROS 2

ROS 2 consists of set of libraries required to build a fully-functioning robot in an isolated environment

### Nodes

ROS is made out of a bunch of smaller programs that all work together to build the final robot. Each of these smaller programs is termed as a node. NOde is designed to carry out operations such as reading sensor data, sending signals to motor and etc.

In this task, I created two nodes:

    A publisher node that sends out data (in this case, numbers and their squares).

    A subscriber node that receives the published data and processes it (displays the square of the number).

### Topics

Topics are the communication channels in ROS 2. It's a named location that allows the publisher nodes to publish the message and the subscriber noded (a node with subscription to the location) to access the message and process it further.

In my example:

    The publisher node publishes messages to a topic (e.g: /number_square).

    The subscriber node listens to this topic and processes the incoming messages.

### Messages

Messages are the structured data formats used by nodes to communicate over topics. These can be simple types like integers or strings, or more complex types defined using .msg files.

I used standard message types (like Int64) in this task to send and receive numerical data. 

### Packages

A ROS 2 package is the basic unit for organizing code. Each package contains all the necessary files — source code, launch files, configuration, and metadata — required to run part of a robotic system.

In this task, I created a Python-based package named number_square_package. This package includes both the publisher and subscriber scripts and the necessary configuration files (setup.py, package.xml, etc.).

### Workspaces

A ROS 2 workspace is a directory where one or more packages can be developed and built. It provides an isolated environment where custom packages can be created, tested, and compiled using build tools like colcon.

I created a workspace called dev_ws, which follows the standard ROS 2 structure with a src directory inside it to store all the packages inside it.

## Structure of my Workspace (dev_ws):

```
dev_ws/
├── build/
├── install/
├── log/
└── src/
    └── number_square_package/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        ├── test/
        └── number_square_package/
            ├── __init__.py
            ├── number_publisher.py
            └── square_subscriber.py
```
## Task Overview

The goal of this task was to build a simple ROS2 system that demonstrates communication between the publisher node and the subscirber node. In my approach, I used a 1 second time delay between each message (number) and the sqaure of that number will be printed in a new terminal dedicated for the subscriber node. I've listed the steps that I followed to successfully complete the project below:

1. Created a new ROS 2 workspace called dev_ws, which follows the standard structure (src/, build/, install/, etc.).

2. Initialized a new ROS 2 package named number_square_package inside the src/ folder using Python as the build type.

3. Developed a publisher node (number_publisher.py) that continuously publishes a stream of integers on a custom topic.

4. Created a subscriber node (square_subscriber.py) that subscribes to the same topic and calculates the square of each received number.

5. Updated the package.xml and setup.py files to include metadata and dependencies required to build and run the nodes.

6. Added an __init__.py file to treat the node scripts as a Python module.

7. Tested the nodes by building the workspace, sourcing the setup script, and running the nodes in separate terminals to verify correct publishing and subscribing behavior.

## Commands

    mkdir -p dev_ws/src
Creates a new ROS 2 workspace directory with a src subdirectory.

    cd dev_ws/src
Navigates into the src folder to create or add packages.

    ros2 pkg create --build-type ament_python number_square_package
Initializes a new Python-based ROS 2 package.

    colcon build
Used to build all the packages in your ROS 2 workspace. It handles package dependencies, builds order, and integrates with CMake or Python build systems.

    source install/setup.bash
Sources the environment so that ROS 2 recognizes your package and nodes.

    ros2 run number_square_package number_publisher
Runs the publisher node from your custom package.

    ros2 run number_square_package square_subscriber
Runs the subscriber node that calculates squares of received numbers.

    ros2 topic list
Lists all active topics, helping you verify if your publisher is working.

    ros2 pkg list
Lists all packages that are discoverable in the current ROS 2 environment.

## Challenges

1. Managing Python Imports Across Packages

Challenge: I faced import errors when trying to access my Python scripts, especially between number_publisher.py and square_subscriber.py.

Fix: I ensured each Python package included an __init__.py file, followed ROS 2 Python module structure, and adjusted the file paths accordingly.

3. Colcon Build Issues

Challenge: The build command failed multiple times due to missing dependencies or syntax errors in package.xml or setup.py.

Fix: I carefully reviewed both configuration files, added required dependencies like rclpy, and fixed YAML/XML formatting issues. After each fix, I rebuilt the workspace using colcon build and sourced the environment properly.

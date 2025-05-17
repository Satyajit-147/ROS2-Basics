# ROS2-Basics

This README documents my initial learning journey with ROS 2 — covering key concepts, essential commands, workspace structure, challenges encountered, and the implementation of basic publisher and subscriber nodes.

## Learnings

This was my first hands-on experience with ROS 2 (Robot Operating System 2), and it gave me a strong foundational understanding of how ROS-based systems are structured and how different components interact. Here's a breakdown of the key concepts I learned and understood during the task:

### ROS 2

ROS 2 consists of set of libraries required to build a fully-functioning robot in an isolated environment

### Nodes

A node is a single process that performs computation. Each node in a ROS 2 system is designed to do a specific task — for example, reading sensor data, controlling motors, or performing some logic. Nodes run independently but can communicate with one another, making systems loosely coupled and easier to manage and debug.

In this task, I created two nodes:

    A publisher node that sends out data (in this case, numbers and their squares).

    A subscriber node that receives the published data and processes it (displays the square of the number).

### Topics

Topics are the communication channels in ROS 2. It's a named location that allows the publisher nodes to publish the message and the subscriber noded (subscribed to the location) to access the message and process it further.

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

I created a workspace called dev_ws, which follows the standard ROS 2 structure with a src directory inside it to store all the packages.

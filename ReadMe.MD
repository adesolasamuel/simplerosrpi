# Read Ultrasonic Sensor on Raspberry Pi using ROS2, display the data on a GUI window

A very simple project…

At least to see ROS2 in actual action… it can be a first step away from the simulation-based common ROS2 classes.

In this guide, we will be reading an ultrasonic sensor connected to a Raspberry Pi using ROS2 and then sending the values to the development computer. I am also thinking of creating a simple Qt Visualizer for the received data. Let's see how it goes.

The Raspberry Pi publisher node will be written in Python, and the subscriber on the development machine will be written in C++. Both the publisher and subscriber could have been written in the same language, but I chose to use Python on the Pi since it's a more common way of interacting with the GPIO pins.

## Prerequisite

Installing ROS2…. What a node is…. what rclpy and rclcpp are… all those things, should not be a big deal for you, I guess. In case of learning more, the official ROS2 documentation (https://docs.ros.org/en/humble/index.html) is the best place to learn about what these whole things are.

Ubuntu 22.04 desktop is installed on my dev computer, and the Ubuntu 22.04 server equivalent is installed on my Raspberry Pi. ROS2 Humble is installed on both. Based on my experience, the distro does not matter.

## Why it all works

ROS2 makes available some very important possibilities via topics, services, and actions. All these enable the exchange of information between/among our robots. The ROS2 domain ID is one thing I found personally exciting about how ROS handles things.

What we are doing in this tutorial is creating a topic that the Raspberry Pi can publish to, and the dev computer can subscribe to. One can think that since the topic name is the same, there should not be any problem for the two machines communicating with each other, but another concept that must be the same is the domain ID of the two machines. On a lighter note, this just allows the two machines to see each other on the network. You can read more on the ROS2 domain ID.

To start with, make sure you run this in the terminal on the two machines. The <your_domain_id> can be any number; just make it the same for the pi and the dev computer.

READ MORE ON MEDIUM: https://samueladesola.medium.com/read-ultrasonic-sensor-on-raspberry-pi-using-ros2-display-the-data-on-a-qt-gui-window-48099271720c
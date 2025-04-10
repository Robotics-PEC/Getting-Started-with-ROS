#########
Overview
#########

Now that you have ROS2 installed on your system, let's first understand the basic concept of ROS2.

ROS2 (Robot Operating System 2) is **not an operating system** like Windows or Linux. Instead, it is a **framework** — a set of software tools and libraries designed to help you build robot applications efficiently.

***************
What is a Framework?
***************

A **framework** provides reusable code, tools, and predefined structures that make software development easier and more organized. In the case of ROS2, it offers the essential building blocks to develop robot software, including message passing, device drivers, simulation tools, and more.

***************
Core Concepts in ROS2
***************

At the heart of ROS2 are a few key concepts:

- **Nodes**: These are independent programs that perform specific tasks. For example, one node might read sensor data, while another controls the wheels.
  
- **Topics**: Nodes communicate with each other by publishing or subscribing to *topics*. A topic is like a named data stream — nodes can publish messages to a topic, and other nodes can subscribe to it to receive those messages.
  
- **Publishers and Subscribers**:
  - A **publisher** is a node that sends data (like camera images or sensor readings) to a topic.
  - A **subscriber** is a node that listens to that topic and processes the incoming data.

***************
Why Are These Necessary?
***************

In robotics, various parts of the system (e.g., sensors, motors, controllers, algorithms) need to work together. ROS2 enables this communication through its publisher-subscriber model. This **decouples** different components, so they can operate independently while still sharing information. This modularity makes your robotic system more **scalable**, **maintainable**, and **reusable**.

***************
How ROS2 Communicates: DDS
***************

Under the hood, ROS2 uses a middleware called **DDS (Data Distribution Service)** to manage communication between nodes. DDS is a standardized protocol that allows real-time, scalable, and reliable data exchange.

Key advantages of DDS include:
    - **Discovery**: Nodes can automatically find each other on the network.
    - **Quality of Service (QoS)**: You can define how messages are sent and received (e.g., best effort vs. reliable, history depth, etc.).
    - **Decentralization**: There's no need for a central server, making systems more robust and scalable.

This DDS-based model means ROS2 applications can be more easily distributed across multiple devices and systems.

***************
Learn More
***************

This is just a high-level overview to get you started. ROS2 is a powerful and flexible framework with many features to explore. To dive deeper, check out the official ROS2 documentation:
`ROS2 nodes explanation <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html>`_

.. note::
   **(Note: Replace "humble" in the link with your ROS2 distro if you're using a different version, e.g., "iron" or "foxy".)**
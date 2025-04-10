##########
Tutorial
##########
This tutorial walks you through running example publisher and subscriber nodes using ROS2. The included repository provides simple, ready-to-run examples written in both Python and C++. These examples are designed to help you:

- Understand how the ROS2 build system (`colcon`) works
- See how nodes communicate using topics
- Observe interoperability between Python and C++ nodes

==========================
Running Talker and Listener
==========================

Each publisher and subscriber in this repository is a **node**, and each node resides within a **package**. This mirrors the modular, scalable architecture that ROS2 encourages for building robotic applications.

Building the code
-----------------

1. **Build the code inside the workspace directory:**

   .. code-block:: bash

      colcon build

2. **Source the setup script inside the install folder:**

   .. code-block:: sh

      source install/setup.bash

Running the nodes
-----------------

1. **Run the Python talker node:**

   .. code-block:: sh

      ros2 run talker_listener_py talker

2. **Run the Python listener node:**

   .. code-block:: sh

      ros2 run talker_listener_py listener

3. **Run the C++ talker node:**

   .. code-block:: sh

      ros2 run talker_listener_cpp talker

4. **Run the C++ listener node:**

   .. code-block:: sh

      ros2 run talker_listener_cpp listener

Experiment with Mixed Language Nodes
------------------------------------

You can also experiment by running nodes written in different languages together. For example:

1. **Run the C++ talker node and the Python listener node:**

   .. code-block:: sh

      ros2 run talker_listener_cpp talker
      ros2 run talker_listener_py listener

2. **Run the Python talker node and the C++ listener node:**

   .. code-block:: sh

      ros2 run talker_listener_py talker
      ros2 run talker_listener_cpp listener

Running the Keystroke Talker
----------------------------

The keystroke talker node allows you to send keystrokes as messages. To run the keystroke talker node, follow these steps:

1.  **Run the keystroke talker node in `C++`:**

   .. code-block:: sh

      ros2 run talker_listner_cpp keyboard_talker_char

2.  **Run the keystroke talker node in `python`:**

   .. code-block:: sh

      ros2 run talker_listner_py keyboard_talker_char

This node will publish keystrokes to a topic, which can be subscribed to by other nodes. This simple interface demonstrates real-time communication and is especially helpful for beginners to see how nodes behave interactively. It will also help you understand how ROS2 nodes written in different languages can communicate with each other over a common topic.

Example Package Overview
------------------------

The repository comes with several **example publisher and subscriber nodes** that you can run immediately to get a feel for how the ROS2 build system works. Each publisher and subscriber is implemented as a **node**, and each node is organized inside a **package**. In ROS2, this package structure is essential for modular design and scalability.

For example:
- `talker_listener_py` is a Python package that contains Python-based nodes
- `talker_listener_cpp` is a C++ package that contains C++-based nodes

Each node communicates by publishing or subscribing to a topic, and these nodes can be run independently or in combination with others. This setup allows you to:

- Observe how ROS2 handles building multi-language packages
- See how the same topic can be used across nodes written in different languages
- Understand how modularity in ROS2 makes code organization and reuse easier

By experimenting with these examples, you gain practical insight into ROS2's architecture, build tools like `colcon`, and how distributed node communication works in real robotic systems.

Try editing one of the talker nodes to publish a custom message and see how the listener reacts. This hands-on trial-and-error approach is a great way to understand the flexibility and power of ROS2.

==========================
Creating Your Own Node
==========================

Once you're comfortable with running existing nodes, you can start creating your own.

Creating a New ROS2 Package
---------------------------

Use the ``ros2 pkg create`` command to generate a new package:

.. code-block:: sh

   ros2 pkg create --build-type ament_cmake my_cpp_package
   ros2 pkg create --build-type ament_python my_python_package

Understanding Build Types
---------------------------

- ``ament_cmake``: Used for C++ packages, leverages CMake for building
- ``ament_python``: Used for Python packages, leverages setuptools and Python packaging

Creating a C++ Package
---------------------------

When using ``--build-type ament_cmake``, ROS2 will set up a CMake project with necessary files like:
   - `CMakeLists.txt`
   - `package.xml`

Place your C++ source files inside a `src/` folder, define your executables in `CMakeLists.txt`, and build using `colcon build`.

.. note::
   If you are unfamiliar with C++, then please go and learn it first, this is not for the faint of heart.

Example:

.. code-block:: cpp

   // my_node.cpp
   #include "rclcpp/rclcpp.hpp"

   int main(int argc, char **argv) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<rclcpp::Node>("my_cpp_node");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }

Creating a Python Package
---------------------------

When using ``--build-type ament_python``, ROS2 will create a Python structure with:
   - `setup.py`
   - `package.xml`
   - `my_python_package/` directory

Place your node scripts inside the package directory, and register them as entry points in `setup.py`.

Example:

.. code-block:: python

   # my_node.py
   import rclpy
   from rclpy.node import Node

   class MyPythonNode(Node):
       def __init__(self):
           super().__init__('my_python_node')
           self.get_logger().info("Hello from Python node!")

   def main(args=None):
       rclpy.init(args=args)
       node = MyPythonNode()
       rclpy.spin(node)
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

For more details and advanced topics, refer to this section of ROS2 documentation:
   - `How to make a publisher in C++ <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_
   - `How to make a publisher in Python <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_


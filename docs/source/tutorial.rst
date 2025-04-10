##########
Tutorial
##########

==========================
Running Talker and Listner
==========================

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

This node will publish keystrokes to a topic, which can be subscribed to by other nodes.
This will help you understand how ROS2 nodes written in different languages can communicate with each other.
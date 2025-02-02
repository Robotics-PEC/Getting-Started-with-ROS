# ROS2 Node in C++: `keyboard_talker_char`

## Overview

This document provides an overview of the `keyboard_talker_char` ROS2 node implemented in C++. The node is designed to read characters from the keyboard and publish them to a ROS2 topic.

## Node Description

### `keyboard_talker_char`

The `keyboard_talker_char` node is responsible for capturing keyboard input and publishing each character as a message to a specified ROS2 topic. This node can be useful for applications that require real-time keyboard input to be communicated to other nodes in a ROS2 system.

### Key Components

1. **Node Initialization**:

   - The node is initialized using the `rclcpp::Node` class, which sets up the ROS2 environment and prepares the node for operation.

2. **Publisher**:

   - A ROS2 publisher is created to send messages of type `std_msgs::msg::Char` to a specified topic. This publisher is responsible for broadcasting the keyboard input to other nodes.

3. **Keyboard Input Handling**:

   - The node captures keyboard input using standard input methods. Each character entered by the user is read and processed.

4. **Message Publishing**:
   - For each character read from the keyboard, a `std_msgs::msg::Char` message is created and populated with the character data. This message is then published to the configured topic.

### Usage

To use the `keyboard_talker_char` node, follow these steps:

1. **Build the Node**:

   - Ensure that your ROS2 workspace is properly set up and that the `keyboard_talker_char` node is included in the CMakeLists.txt and package.xml files.
   - Build the workspace using `colcon build`.

2. **Run the Node**:

   - Source the ROS2 setup script: `source install/setup.bash`.
   - Launch the node using the ROS2 run command: `ros2 run <package_name> keyboard_talker_char`.

3. **Interact with the Node**:
   - Once the node is running, type characters into the terminal. Each character will be published to the specified ROS2 topic.

### Example

Here is an example of how to run the `keyboard_talker_char` node and observe the published messages:

1. Open a terminal and run the `keyboard_talker_char` node:

   ```sh
   ros2 run <package_name> keyboard_talker_char
   ```

2. Open another terminal and use `ros2 topic echo` to listen to the topic:

   ```sh
   ros2 topic echo /keyboard_input
   ```

3. Type characters in the first terminal and observe the messages being published and echoed in the second terminal.

## Conclusion

The `keyboard_talker_char` node is a simple yet effective way to capture and publish keyboard input in a ROS2 system. By following the steps outlined in this document, you can integrate this node into your ROS2 applications and leverage real-time keyboard input for various purposes.

## Creating Your Own ROS2 Node in C++

To create your own ROS2 node in C++, follow these steps:

### 1. Set Up Your ROS2 Workspace

First, ensure you have a ROS2 workspace set up. If you don't have one, you can create it as follows:

```sh
mkdir -p ~/workspace/src
cd ~/workspace
colcon build
source install/setup.bash
```

### 2. Create a New Package

Create a new ROS2 package using the `ros2 pkg create` command:

```sh
cd ~/workspace/src
ros2 pkg create --build-type ament_cmake my_cpp_node
```

This command creates a new package named `my_cpp_node` with the necessary files and directories.

### 3. Write Your Node

Navigate to the `src` directory of your package and create a new C++ file for your node:

```sh
cd ~/workspace/src/my_cpp_node/src
touch my_node.cpp
```

Edit `my_node.cpp` with your preferred text editor and add the following code:

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

### 4. Update CMakeLists.txt

Edit the `CMakeLists.txt` file in your package directory to include your new node:

```cmake
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})
```

### 5. Build Your Package

Navigate back to your workspace root and build your package:

```sh
cd ~/workspace
colcon build
source install/setup.bash
```

### 6. Run Your Node

You can now run your node using the `ros2 run` command:

```sh
ros2 run my_cpp_node my_node
```

You should see the message "Hello, ROS2!" printed in the terminal, indicating that your node is running successfully.

By following these steps, you can create and run your own ROS2 nodes in C++. This basic example can be expanded with publishers, subscribers, services, and more to build complex ROS2 applications.

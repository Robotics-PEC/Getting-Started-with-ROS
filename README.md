# Getting-Started-with-ROS

This repository is designed to teach ROS2 (Robot Operating System 2) for beginners using the Jazzy distribution.

## Getting Started

### Prerequisites
- Docker
- Xming (for Windows users)

### Setup

1. **Start Docker:**
    ```sh
    docker-compose up -d
    ```

2. **Attach to Docker container:**
    ```sh
    docker-compose exec ros bash
    ```

### XServer Setup (Windows)
To forward video from Docker to the base OS, ensure you have Xming installed and running.

## Running Talker and Listner

### Building the code
1. **Build the code inside the workspace directory:**
    ```sh
    colcon build
    ```

2. **Source the setup script inside the install folder:**
    ```sh
    source install/setup.bash
    ```

### Running the nodes
1. **Run the Python talker node:**
    ```sh
    ros2 run talker_listener_py talker
    ```

2. **Run the Python listener node:**
    ```sh
    ros2 run talker_listener_py listener
    ```

3. **Run the C++ talker node:**
    ```sh
    ros2 run talker_listener_cpp talker
    ```

4. **Run the C++ listener node:**
    ```sh
    ros2 run talker_listener_cpp listener
    ```

### Experiment with Mixed Language Nodes

You can also experiment by running nodes written in different languages together. For example:

1. **Run the C++ talker node and the Python listener node:**
    ```sh
    ros2 run talker_listener_cpp talker
    ros2 run talker_listener_py listener
    ```

2. **Run the Python talker node and the C++ listener node:**
    ```sh
    ros2 run talker_listener_py talker
    ros2 run talker_listener_cpp listener
    ```

This will help you understand how ROS2 nodes written in different languages can communicate with each other.

## License
This project is licensed under the [GNU General Public License v3.0](LICENSE).
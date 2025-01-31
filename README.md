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

## License
This project is licensed under the [GNU General Public License v3.0](LICENSE).
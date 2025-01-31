#!/bin/bash

# Check if the ROS setup script is already in .bashrc
if ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc
fi

# Execute the passed command
exec "$@"

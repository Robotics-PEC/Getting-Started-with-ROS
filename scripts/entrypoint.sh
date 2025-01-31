#!/bin/bash

# Source the ROS setup script
source /opt/ros/jazzy/setup.bash

# Add the ROS setup script to .bashrc for future sessions
echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc

# Execute the passed command
exec "$@"

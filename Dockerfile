# Use the official ROS Jazzy desktop image as the base image
FROM osrf/ros:jazzy-desktop

# Update the package list and install sudo
RUN apt-get update && apt-get install -y sudo

# Create a new user 'jazzer' with password 'password' and add to sudoers
RUN useradd -m jazzer && echo "jazzer:password" | chpasswd && adduser jazzer sudo

# Copy the entrypoint script to the home directory of the new user
COPY scripts/entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Switch to the new user 'jazzer'
USER jazzer

# Set the working directory for the new user
WORKDIR /home/jazzer/workspace

# Set the entry point to the script
CMD ["/bin/bash"]
ENTRYPOINT ["/entrypoint.sh"]
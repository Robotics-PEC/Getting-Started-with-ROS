# Use the official ROS Jazzy desktop image as the base image
FROM osrf/ros:jazzy-desktop

# Update the package list and upgrade installed packages
RUN apt-get update && apt-get upgrade -y

# Update package list again before installing sudo (ensures latest repo metadata)
RUN apt-get update && apt-get install -y sudo

# Create a new user 'jazzer' without a password and add to sudoers
RUN useradd -m jazzer && usermod -aG sudo jazzer

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

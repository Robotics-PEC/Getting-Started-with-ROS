# Use the official ROS Jazzy desktop image as the base image
FROM osrf/ros:jazzy-desktop

# Update the package list and upgrade installed packages
RUN apt-get update && apt-get upgrade -y

# Install sudo
RUN apt-get update && apt-get install -y sudo

# Create a new user 'jazzer' and add to sudo group
RUN useradd -m jazzer && usermod -aG sudo jazzer

# Allow passwordless sudo for the jazzer user
RUN echo "jazzer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/jazzer && chmod 0440 /etc/sudoers.d/jazzer

# Copy the entrypoint script
COPY scripts/entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Switch to user
USER jazzer

# Set the working directory
WORKDIR /home/jazzer/workspace

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

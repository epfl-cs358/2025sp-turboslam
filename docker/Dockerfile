# Use the official ROS 2 Humble base image on Ubuntu Jammy
FROM ros:humble-ros-base-jammy

# Use bash for RUN commands to allow sourcing
SHELL ["/bin/bash", "-lc"]

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools, micro-ROS dependencies, PlatformIO, and utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-dev-tools \
    python3-vcstool \
    git \
    curl \
    python3-pip \
    udev \
  && pip3 install platformio \
  && rm -rf /var/lib/apt/lists/*

# Prepare and build the micro-ROS Agent workspace
RUN mkdir -p /microros_ws/src && cd /microros_ws \
  && git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
  && apt-get update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -y \
  && source /opt/ros/humble/setup.sh \
  && colcon build \
  && source install/local_setup.sh \
  && ros2 run micro_ros_setup create_agent_ws.sh \
  && ros2 run micro_ros_setup build_agent.sh \
  && rm -rf /var/lib/apt/lists/*

# Set default workspace for user code
WORKDIR /workspace

# Auto-source ROS 2 and micro-ROS Agent on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
  && echo "source /microros_ws/install/local_setup.sh" >> /root/.bashrc

CMD ["bash"]
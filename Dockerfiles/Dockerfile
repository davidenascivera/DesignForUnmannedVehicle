# Use the official ROS2 Humble Desktop image as the base
FROM osrf/ros:humble-desktop

# Set environment variables to avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Update package lists and install necessary dependencies
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    wget \
    git \
    ca-certificates \
    build-essential \
    cmake \
    python3-pip \
    sudo \
    gnupg2 \
    lsb-release \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libfuse2 \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Create a non-root user named 'user' with sudo privileges
RUN useradd -m -s /bin/bash user && \
    echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/user

# Add the user to the 'dialout' group for serial port access
RUN usermod -aG dialout user

# Switch to the non-root user
USER user
WORKDIR /home/user

# Install Python dependencies
RUN pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools

# Clone the PX4-Autopilot repository recursively
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install cerberus and coverage
RUN pip3 install --user cerberus coverage

# Downgrade setuptools to avoid issues
RUN pip3 install --user setuptools==58.0.4

# Navigate to the PX4-Autopilot directory and run the setup script
RUN cd PX4-Autopilot && \
    bash ./Tools/setup/ubuntu.sh

# Create a workspace for combined sensors and clone necessary repositories
RUN mkdir -p ~/ws_combined_sensor/src/ && \
    cd ~/ws_combined_sensor/src/ && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    git clone https://github.com/PX4/px4_msgs.git


# Clone, build, and install Micro-XRCE-DDS-Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/


RUN echo "source /opt/ros/humble/setup.bash"  >> ~/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]

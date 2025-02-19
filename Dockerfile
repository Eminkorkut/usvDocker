FROM ubuntu:22.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Set the working directory
WORKDIR /usv

# Install necessary dependencies
RUN apt update && apt install -y \
    locales \
    software-properties-common \
    curl \
    gazebo \
    python3 \
    python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && apt update \
    && apt upgrade -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    opencv-python \
    ultralytics \
    matplotlib \
    pynput \
    numpy==1.26.4 \
    torch

# Install additional ROS and system packages
RUN apt install -y ros-humble-gazebo-ros-pkgs plocate \
    && updatedb \
    && echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib" >> /root/.bashrc

# Copy all files from the current directory to /usv, excluding Dockerfile
COPY . /usv/

RUN apt-get install -y libgl1-mesa-glx libegl1-mesa

# Source ROS 2 setup script
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# Set the default command to bash
CMD ["/bin/bash"]

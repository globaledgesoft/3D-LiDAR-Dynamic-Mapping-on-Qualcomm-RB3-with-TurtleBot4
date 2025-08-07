FROM arm64v8/ubuntu:20.04
 
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata
 
# --- Step 1: Base setup ---
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    locales && \
    locale-gen en_US.UTF-8
 
# --- Step 2: ROS Noetic setup ---
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-noetic-desktop-full
 
# --- Step 3: ROS dependencies ---
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs
 
# --- Step 4: Initialize rosdep ---
RUN rosdep init && rosdep update
 
# --- Step 5: Environment sourcing ---
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash
 
# --- Step 6: Clone Unitree Lidar SDK ---
WORKDIR /home
RUN git clone https://github.com/unitreerobotics/unilidar_sdk.git
 
# --- Step 7: Build ROS package ---
WORKDIR /home/unilidar_sdk/unitree_lidar_ros
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
 
# --- Step 8: Source workspace on container start ---
RUN echo "source /home/unilidar_sdk/unitree_lidar_ros/devel/setup.bash" >> ~/.bashrc
 
# --- Step 9: Set working dir ---
WORKDIR /home/unilidar_sdk/unitree_lidar_ros
 
CMD ["/bin/bash"]

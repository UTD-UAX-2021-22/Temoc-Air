# Build:
#  docker build --rm -f Dockerfile -t gramaziokohler/ros-noetic-base .
#
# Usage:
#  docker pull gramaziokohler/ros-noetic-base
FROM nvcr.io/nvidia/l4t-base:r32.6.1
# FROM stereolabs/zed:3.7-devel-l4t-r32.6.1
LABEL maintainer "Noah Parker <noah-parker@live.com>"

SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND="noninteractive"

# All apt-get commands start with an update, then install
# and finally, a cache cleanup to keep the image size small.
RUN apt-get update
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install curl gnupg2 -y # if you haven't already installed curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install ros-melodic-desktop -y



RUN source /opt/ros/melodic/setup.bash \
    && apt-get update \
    && apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# Install packages
RUN apt-get update \
    && apt-get install -y \
    # Basic utilities
    git \
    net-tools \ 
    iputils-ping \
    dnsutils \
    wget 
    # ROS bridge server and related packages
    # ros-${ROS_DISTRO}-rosbridge-server \
    # ros-${ROS_DISTRO}-tf2-web-republisher   # NOT yet published?   \
    #--no-install-recommends \
    # && rm -rf /var/lib/apt/lists/*

RUN wget -O ZED_Install.run https://download.stereolabs.com/zedsdk/3.7/jp46/jetsons
RUN chmod +x ZED_Install.run
RUN ./ZED_Install.run -- silent

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN source /opt/ros/melodic/setup.bash \
    && git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
   # && cd ../ \
RUN source /opt/ros/melodic/setup.bash \ 
    && rosdep init \
    && rosdep update
RUN source /opt/ros/melodic/setup.bash \    
    && pwd \
    && cd .. \
    && apt-get install ros-melodic-tf2-geometry-msgs apt libusb-1.0-0-dev -y \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make -DCMAKE_BUILD_TYPE=Release \
    && source ./devel/setup.bash


# Create local catkin workspace
ARG CACHEBUST=0 

RUN ls zed-ros-wrapper/zed_wrapper/params
COPY common.yaml zed-ros-wrapper/zed_wrapper/params/
COPY zed2.yaml zed-ros-wrapper/zed_wrapper/params/

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /usr/local/bin/ros_catkin_entrypoint.sh" >> /root/.bashrc

# COPY roscore_check.py /usr/local/bin/roscore_check.py
COPY ros_catkin_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint.sh

# RUN chmod +x /usr/local/bin/roscore_check.py
RUN chmod +x /usr/local/bin/ros_catkin_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint.sh"]
CMD ["bash"]

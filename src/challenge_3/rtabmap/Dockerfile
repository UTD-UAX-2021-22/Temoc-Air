# Build:
#  docker build --rm -f Dockerfile -t gramaziokohler/ros-noetic-base .
#
# Usage:
#  docker pull gramaziokohler/ros-noetic-base
#
# Based of gramaziokohler/ros-noetic-base

FROM ros:noetic
ENV DEBIAN_FRONTEND="noninteractive"
LABEL maintainer "Noah Parker <noah-parker@live.com>"

SHELL ["/bin/bash","-c"]

# All apt-get commands start with an update, then install
# and finally, a cache cleanup to keep the image size small.

# Install packages
RUN apt-get update \
    && apt-get install -y \
    # Basic utilities
    git \
    net-tools \ 
    iputils-ping \
    dnsutils

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    # ROS File Server
    && git clone https://github.com/gramaziokohler/ros_file_server.git \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get install ros-${ROS_DISTRO}-rtabmap-ros -y



RUN source /opt/ros/${ROS_DISTRO}/setup.bash \    
    && pwd \
    && cd .. \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make -DCMAKE_BUILD_TYPE=Release \
    && source ./devel/setup.bash


# Create local catkin workspace

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /usr/local/bin/ros_catkin_entrypoint.sh" >> /root/.bashrc

# COPY roscore_check.py /usr/local/bin/roscore_check.py
COPY ros_catkin_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint.sh

# RUN chmod +x /usr/local/bin/roscore_check.py
RUN chmod +x /usr/local/bin/ros_catkin_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint.sh"]
CMD ["bash"]

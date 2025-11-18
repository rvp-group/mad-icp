ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher
ARG OVERLAY_WS

# overwrite defaults to persist minimal cache
RUN rosdep update --rosdistro $ROS_DISTRO && \
  cat <<EOF > /etc/apt/apt.conf.d/docker-clean && apt-get update
APT::Install-Recommends "false";
APT::Install-Suggests "false";
EOF


RUN apt-get install -y \
  libeigen3-dev \
  python3-venv \
  python3-pip \
  ros-humble-rmw-cyclonedds-cpp \
  && \
  rm -rf /var/lib/apt/lists/*


# clone overlay source

WORKDIR $OVERLAY_WS/src
COPY . ./

ENV RWM_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR $OVERLAY_WS
# setup mad-icp python package
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#   pip install src/

# build ros2 packages 
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  colcon build




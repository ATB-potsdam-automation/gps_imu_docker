ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG ROS_DISTRO=humble


# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
ARG ROS_DISTRO
WORKDIR $OVERLAY_WS/src

RUN echo "\
repositories: \n\
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
ARG ROS_DISTRO
WORKDIR $OVERLAY_WS

COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/ATB-potsdam-automation/atb_ublox_gps.git
RUN git clone https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver.git -b ros2_0_foxy
RUN git clone https://github.com/ATB-potsdam-automation/ntrip_client.git -b ros2
RUN git clone https://github.com/cra-ros-pkg/robot_localization.git -b ros2
RUN git clone https://github.com/ros/robot_state_publisher.git -b ros2
RUN git clone https://github.com/KumarRobotics/ublox.git -b ros2 

WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update 
run rosdep install -y \
      --from-paths . \
        ./src/atb_ublox_gps \
        ./src/bluespace_ai_xsens_ros_mti_driver \
        ./src/ntrip_client \
        ./src/robot_localization \
        ./src/ublox/ublox_gps \
        ./src/ublox/ublox \
        ./src/ublox/ublox_msgs \
        ./src/ublox/ublox_serialization \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
ARG ROS_DISTRO
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --packages-skip bluespace_ai_xsens_mti_driver \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh
RUN sudo usermod -G dialout -a root 
RUN newgrp dialout
RUN apt-get update && apt-get install -y iputils-ping
RUN apt-get install -y python3-pip
RUN pip3 install xacro
# run launch file
#CMD ["ros2", "launch", "atb_ublox_gps", "docker_final_launch.py"]
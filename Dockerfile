# Copyright 2019 Mikael Arguedas
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ARG FROM_IMAGE=osrf/ros2:devel
FROM $FROM_IMAGE

ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO

# install building tools
RUN apt-get -qq update && \
    apt-get -qq upgrade -y && \
    if [ -e /opt/ros/$ROS_DISTRO/setup.bash ]; then true; else apt-get -qq install ros-$ROS_DISTRO-ros-workspace -y; fi && \
    rm -rf /var/lib/apt/lists/*

ARG REPO_SLUG=repo/to/test
ARG CI_FOLDER=.ros2ci

# setup underlay
ENV ROS2_UNDERLAY_WS /opt/ros2_underlay_ws
# copy optional additional_repos.repos
COPY ./$CI_FOLDER/additional_repos.repos $ROS2_UNDERLAY_WS/
RUN mkdir -p $ROS2_UNDERLAY_WS/src
WORKDIR $ROS2_UNDERLAY_WS
RUN if [ -f additional_repos.repos ]; then vcs import src < additional_repos.repos; fi
# build underlay
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon \
    build \
    --merge-install \
    --cmake-args -DSECURITY=ON -DBUILD_TESTING=OFF --no-warn-unused-cli

ENV ROS_PACKAGE_PATH=$ROS2_UNDERLAY_WS/install/share:$ROS_PACKAGE_PATH

# setup overlay
ENV ROS2_OVERLAY_WS /opt/ros2_overlay_ws
RUN mkdir -p $ROS2_OVERLAY_WS/src/$REPO_SLUG
COPY ./$CI_FOLDER/*.bash $ROS2_OVERLAY_WS/
WORKDIR $ROS2_OVERLAY_WS

# setup entrypoint
COPY ./$CI_FOLDER/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ['bash']

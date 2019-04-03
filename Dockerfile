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

ARG ROS_DISTRO=crystal
ENV ROS_DISTRO=$ROS_DISTRO

# install building tools
RUN apt-get -qq update && \
    apt-get -qq upgrade -y && \
    apt-get -qq install ros-$ROS_DISTRO-ros-workspace -y && \
    rm -rf /var/lib/apt/lists/*

ARG REPO_SLUG=repo/to/test
ARG CI_FOLDER=.ros2ci

ENV ROS2_OVERLAY_WS /opt/ros2_overlay_ws
# underlay workspace
# clone underlay
ENV ROS2_UNDERLAY_WS /opt/ros2_underlay_ws
# copy optional ci_script.bash and additional_repos.repos to workspace
COPY ./$CI_FOLDER/*.bash* ./$CI_FOLDER/additional_repos.repos* $ROS2_OVERLAY_WS/
RUN mkdir -p $ROS2_UNDERLAY_WS/src
WORKDIR $ROS2_UNDERLAY_WS
RUN if [ -f additional_repos.repos ]; then vcs import $ROS2_UNDERLAY_WS/src < $ROS2_OVERLAY_WS/additional_repos.repos; fi
# build underlay
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon \
    build \
    --cmake-args -DSECURITY=ON -DBUILD_TESTING=OFF --no-warn-unused-cli \
    --symlink-install
ENV ROS2_UNDERLAY_SETUP $ROS2_UNDERLAY_WS/install/setup.sh
# clone overlay
RUN mkdir -p $ROS2_OVERLAY_WS/src/$REPO_SLUG
WORKDIR $ROS2_OVERLAY_WS

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ['bash']

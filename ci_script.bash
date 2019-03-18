#!/bin/bash
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

set -e

cd $ROS2_WS

# install dependencies
rosdep update && apt-get -qq update && rosdep install -y \
  --from-paths src \
  --ignore-src \
  --rosdistro $ROS_DISTRO \

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# build
colcon build \
    --symlink-install \
    --cmake-args -DSECURITY=ON --no-warn-unused-cli
# test
colcon test \
    --executor sequential \
    --event-handlers console_direct+
colcon test-result

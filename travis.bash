# !/bin/bash
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

if [[ "$1" != "crystal" ]] && [[ "$1" != "dashing" ]] && [[ "$1" != "eloquent" ]] && [[ "$1" != "nightly" ]]; then
  echo "'$1' distro not supported"
  exit -1;
elif [[ "$1" == "nightly" ]]; then
  export base_image="osrf/ros2:nightly";
  export ros_distro="foxy"
else
  export base_image="osrf/ros2:devel";
  export ros_distro="$1"
fi

export distro="$1"

docker build -f .ros2ci/Dockerfile -t ${TRAVIS_REPO_SLUG,,}:$distro --build-arg REPO_SLUG=${TRAVIS_REPO_SLUG} --build-arg FROM_IMAGE=$base_image --build-arg ROS_DISTRO=$ros_distro .

docker run -v ${TRAVIS_BUILD_DIR}:/opt/ros2_overlay_ws/src/${TRAVIS_REPO_SLUG} ${TRAVIS_REPO_SLUG,,}:$distro /opt/ros2_overlay_ws/ci_script.bash

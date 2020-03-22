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
  export dockerfile=".ros2ci/Dockerfile"
elif [[ "$1" == "source" ]]; then
  export base_image="osrf/ros2:devel";
  export ros_distro="foxy"
  export dockerfile=".ros2ci/Dockerfile.source"
else
  if [[ "$1" == "crystal" ]] || [[ "$1" == "dashing" ]] || [[ "$1" == "eloquent" ]]; then
    export base_image="osrf/ros2:devel-bionic";
    export ros_distro="$1"
    export dockerfile=".ros2ci/Dockerfile"
  else
    export base_image="osrf/ros2:devel";
    export ros_distro="$1"
    export dockerfile=".ros2ci/Dockerfile"
  fi
fi

if [[ -n ${GITHUB_ACTIONS} ]]; then
  event_type=${GITHUB_EVENT_NAME}
  repo_slug=${GITHUB_REPOSITORY}
  workspace=${GITHUB_WORKSPACE}
elif [[ -n ${TRAVIS} ]]; then
  event_type=${TRAVIS_EVENT_TYPE}
  repo_slug=${TRAVIS_REPO_SLUG}
  workspace=${TRAVIS_BUILD_DIR}
else
  echo "onky supporting travis and github actions at the moment"
  return -1;
fi

export distro="$1"

docker build -f $dockerfile -t ${repo_slug,,}:$distro --build-arg REPO_SLUG=${repo_slug} --build-arg FROM_IMAGE=$base_image --build-arg ROS_DISTRO=$ros_distro .

docker run -v ${workspace}:/opt/ros2_overlay_ws/src/${repo_slug} ${repo_slug,,}:$distro /opt/ros2_overlay_ws/ci_script.bash

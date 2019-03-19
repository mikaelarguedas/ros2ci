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

if [[ "$1" != "crystal" ]] && [[ "$1" != "nightly" ]]; then
  echo "'$1' distro not supported"
  exit -1;
fi

export distro="$1"

docker build -f .ros2ci/Dockerfile.$distro -t ${TRAVIS_REPO_SLUG,,}:$distro --build-arg REPO_SLUG=${TRAVIS_REPO_SLUG} .

docker run -v ${TRAVIS_BUILD_DIR}:/root/ros2_ws/src/${TRAVIS_REPO_SLUG} ${TRAVIS_REPO_SLUG,,}:$distro /root/ros2_ws/ci_script.bash

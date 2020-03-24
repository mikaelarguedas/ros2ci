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

function install_dependencies() {
# install dependencies
apt-get -qq update && \
rosdep update --rosdistro $ROS_DISTRO && \
rosdep install -y \
  --from-paths src \
  --ignore-src \
  --rosdistro $ROS_DISTRO
}

function build_workspace() {
cmd="
colcon build \
    --event-handlers console_package_list+ \
    --symlink-install \
    --cmake-args $COLCON_EXTRA_CMAKE_ARGS --no-warn-unused-cli \
    --mixin $MIXIN_BUILD build-testing-on \
    ${COLCON_BUILD_EXTRA_ARGS}
"
echo "$cmd"
$cmd

# colcon build \
#     --event-handlers console_package_list+ \
#     --symlink-install \
#     --cmake-args $COLCON_EXTRA_CMAKE_ARGS --no-warn-unused-cli \
#     --mixin $MIXIN_BUILD build-testing-on \
#     ${COLCON_BUILD_EXTRA_ARGS}
# 
}

function test_workspace() {

cmd="
colcon test \
    --pytest-args $COLCON_EXTRA_PYTEST_ARGS \
    --executor sequential \
    --mixin $MIXIN_TEST \
    --event-handlers console_direct+ \
    ${COLCON_TEST_EXTRA_ARGS}
"
echo "$cmd"
$cmd
# colcon test \
#     --pytest-args $COLCON_EXTRA_PYTEST_ARGS \
#     --executor sequential \
#     --mixin $MIXIN_TEST \
#     --event-handlers console_direct+ \
#     ${COLCON_TEST_EXTRA_ARGS}
# use colcon test-result to get list of failures and return error code accordingly
colcon test-result
}

function setup_coverage() {

  export MIXIN_BUILD="$MIXIN_BUILD coverage-gcc "
  # uncomment once https://github.com/colcon/colcon-mixin-repository/pull/21 is merged
  #  export MIXIN_TEST="$MIXIN_TEST coverage-py"
  export COLCON_TEST_EXTRA_ARGS="$COLCON_TEST_EXTRA_ARGS --pytest-with-coverage"
  export COLCON_EXTRA_PYTEST_ARGS="$COLCON_EXTRA_PYTEST_ARGS --cov-report=term"

  apt -qq update && apt -qq install -y curl
  # # install coverage deps
  # pip3 install --user colcon-lcov-result
  # apt -qq update && apt -qq install -y lcov
  # # initialize lcov ?
  # colcon lcov-result --initial
}

install_dependencies

# source ROS_DISTRO in case newly installed packages modified environment
source /opt/ros/$ROS_DISTRO/setup.bash

if [[ -n "${COVERAGE}" ]]; then
  setup_coverage
fi

build_workspace
source install/setup.bash
test_workspace

if [[ -n "${COVERAGE}" ]]; then
  # colcon lcov-result
  # copy all coverage files outside container for upload
  cd $ROS2_OVERLAY_WS/build
  echo "copying coverage report files"
  for file in $(find -name coverage.xml)
  do
    mkdir -p $(dirname $ROS2_OVERLAY_WS/coverage/$file)
    cp $file $ROS2_OVERLAY_WS/coverage/$file
  done
fi

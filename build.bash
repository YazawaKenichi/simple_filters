#!/bin/bash

pushd ~/microros_ws
rm -rf install/simple_lifecycle build/simple_lifecycle
colcon build --packages-select simple_lifecycle --symlink-install
source install/setup.bash
ros2 launch simple_lifecycle simple_lifecycle_launch.py
popd



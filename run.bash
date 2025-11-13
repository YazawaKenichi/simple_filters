#!/bin/bash

pushd ~/microros_ws
source install/setup.bash
ros2 launch simple_lifecycle simple_lifecycle_launch.py
popd



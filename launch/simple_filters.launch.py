#!/usr/bin/env python3
# coding : utf-8
# SPDX-FileCopyrightText: 2023 YAZAWA Kenichi <s21c1036hn@gmail.com>
# SPDX-License-Identifier: MIT-LICENSE
# myname_pubsub > myname_pubsub_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # $HOME/ros2_ws/install/myname_pubsub/share/myname_pubsub/ + config/ + myname_pubsub.param.yaml
    config = os.path.join(get_package_share_directory("simple_filters"), "config", "simple_filters.param.yaml")
    return LaunchDescription([
        Node(package = "simple_filters", executable = "caster_i2f", output = "screen", parameters = [config]),
        Node(package = "simple_filters", executable = "average_filter", output = "screen", parameters = [config]),
        Node(package = "simple_filters", executable = "median_filter", output = "screen", parameters = [config]),
        Node(package = "simple_filters", executable = "distributed_filter", output = "screen", parameters = [config]),
        Node(package = "simple_filters", executable = "derivative_filter", output = "screen", parameters = [config]),
        ])


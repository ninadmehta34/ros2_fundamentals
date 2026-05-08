# ROS2 Actions Quiz Navigation with Custom Action Interface

A ROS2 implementation of an Action Server and Client that navigates a robot to a target coordinate using the ROS2 Navigation Stack, while reporting distance metrics in real time.

## Overview
This project was built as part of The Construct's ROS2 Fundamentals course. It demonstrates how to create a custom action interface, an action server, and an action client that work together to navigate a robot to a goal position on a simulated Mars terrain.

## Demo


https://github.com/user-attachments/assets/bb19694b-f0a5-42bb-925a-ce33c54aaccb

## Features

- Navigates the robot to goal coordinates (8.3, -2.2, -0.2) using the ROS2 Nav Stack
- Publishes distance left to goal as feedback on `/distance_left`
- Returns total distance traveled and a `True` success flag upon completion
- Subscribes to `/odom` for real-time robot position tracking



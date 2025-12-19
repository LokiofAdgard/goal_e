# goal_e — ROS2 Ball‑Collecting Robot

## Introduction
The **goal_e** package implements a ROS2‑based mobile robot designed to autonomously locate, approach, collect, and deliver three balls of different radii and colors to a designated goal area. Although the balls differ visually, the robot relies exclusively on LiDAR‑based geometric detection rather than color or image processing. The system integrates a custom URDF model, ros2_control hardware interfaces, SLAM‑based mapping, Nav2 navigation, and a two‑node automation system that separates perception from action. The robot platform consists of a four‑wheel base with differential‑drive locomotion and two front‑mounted L‑shaped arms that collect targets. Together, these components form a autonomous system capable of performing repeated ball‑collection cycles within a mapped environment. System proved to be capable of acomplishing task but had a very low success rate of collecting all 3 balls at roughly 20 %.

## Objective
The primary objective is to autonomously collect three balls, each with a different radius and color, and transport them to a predefined goal zone. Because the robot uses only a 2D LiDAR sensor, the system distinguishes balls solely through their geometric signatures in the LiDAR scan.

**Ball characteristics:**
- Three balls total  
- Three different radii  
- Three different colors

The robot must detect each ball, compute an approach pose, collect it using the arms, and finally deliver all three to the goal area.

## System Architecture
The software architecture is built around two primary ROS2 nodes:

**1. Ball Detection & Approach‑Point Node**
- Subscribes to LiDAR scans  
- Performs geometric clustering  
- Estimates ball radius and position  
- Computes approach poses  
- Publishes ball pose + approach pose  

**2. Collection & Goal‑Placement Node**
- Subscribes to approach poses  
- Sends Nav2 goals  
- Executes arm‑sweeping trajectories  
- Advances robot to secure ball  
- Tracks number of collected balls  
- Navigates to goal area and releases balls  

This separation ensures modularity and simplifies debugging.

## URDF Model
The robot is described using a modular Xacro‑based URDF that defines the chassis, wheels, arms, and sensor placement. The chassis uses a four‑wheel configuration, with the rear wheels actuated through a differential‑drive setup and the front wheels acting as passive casters. This design improves stability during collection, especially when the robot performs small corrective motions near a ball.

**Defined wheel joints:**
- `chassis_to_lf_wheel`  
- `chassis_to_lb_wheel`  
- `chassis_to_rf_wheel`  
- `chassis_to_rb_wheel`  

**Arm joints:**
- `chassis_to_arm1`  
- `chassis_to_arm2`  

The URDF also includes collision geometry, inertial parameters, and a LiDAR mounting point.

## ros2_control Integration
The robot uses ros2_control to interface with both the wheels and the collection arms.

**Controllers:**
- `diff_drive_controller` — wheel actuation  
- `joint_trajectory_controller` — arm sweeping motions  

Odometry is derived solely from wheel encoders, which introduces drift but remains adequate for short‑range approach maneuvers.

## SLAM Mapping
Mapping is performed using a dedicated SLAM launch file (e.g., SLAM Toolbox). The robot is teleoperated through the environment to build a 2D occupancy map, which is then saved and later used by Nav2 for localization and path planning. The LiDAR‑only sensing configuration provides reliable SLAM performance with clean wall boundaries and stable localization.

## Teleoperation vs. Nav2 Navigation
Teleoperation and Nav2 are implemented as **separate modes**, each with different strengths.

**Teleoperation mode:**
- Direct manual control  
- Highly precise alignment  
- Easily completes the full 3‑ball objective  
- Not affected by localization drift  

**Nav2 autonomous mode:**
- Uses global + local planners  
- Works well in open areas  
- Struggles with precise alignment near small objects  
- More prone to drift and oscillations  

In practice, pure teleop can reliably achieve the goal, while Nav2 is less successful due to the precision required for ball collection.

## Ball Detection and Collection
Ball detection relies entirely on LiDAR‑based geometric clustering. The detection node identifies clusters matching expected radii and estimates their positions. Once a ball is detected, the node computes an approach pose that positions the robot at a fixed offset and orientation relative to the ball. The collection node then navigates to this pose and executes the sweeping routine, advancing slightly while the arms close inward to guide the ball into the holding area.

## Failure Conditions
Several conditions can cause the autonomous pipeline to fail:

**Detection‑related failures:**
- Balls too close to walls or obstacles  
- Balls partially occluded  
- Balls at shallow LiDAR angles  
- Balls moving or rolling after detection  

**Navigation‑related failures:**
- Nav2 unable to reach the exact approach pose  
- Local planner oscillations near tight spaces  
- Odometry drift causing misalignment  

**Collection‑related failures:**
- Incorrect approach angle  
- Ball slipping due to floor friction  
- Arms missing the ball due to small pose errors  

These limitations highlight the challenges of LiDAR‑only perception and precise manipulation using a navigation stack designed for larger‑scale motion.

## Goal‑Area Placement
After collecting all three balls, the robot navigates to a predefined goal area. Upon arrival, the arms open fully and the robot reverses slightly to release the balls. The arms then return to their neutral position, completing the collection cycle.

## Discussion
The **goal_e** system demonstrates a complete ROS2 robotics pipeline, integrating URDF modeling, ros2_control hardware abstraction, SLAM‑based mapping, Nav2 navigation, and custom perception and manipulation logic. The robot successfully performs mapping, navigation, teleoperation, and basic ball‑collection behaviors. However, the final autonomous application was only partially successful. Teleoperation consistently achieves the full objective, while Nav2 struggles with the precision required for interacting with small, movable objects. Despite these challenges, the system provides a functional prototype of an autonomous ball‑collecting robot built entirely on ROS2 and establishes a strong foundation for future improvements.

## Links
- GitHub Repository (placeholder)  
- Demonstration Video (placeholder)

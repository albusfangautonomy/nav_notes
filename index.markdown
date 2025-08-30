---
layout: default
title: Navigation Notes
nav_order: 2
has_children: true
---

# 🧭 Navigation Notes

Welcome to my collection of navigation-related notes. These pages compile theory, algorithms, and practical techniques for localization, mapping, and motion planning.

## 📚 Core Topics

### Localization

- [Overview](./notes/overview.html) \
  Fundamental concepts in navigation: frames of reference, coordinate transforms, and reference systems.

- [Dead Reckoning](./notes/dead_reckoning.html) \
  Using onboard odometry and motion models to estimate position over time.

- [Kalman Filters](./notes/kalman_filter.html) \
  Linear Kalman Filter, Extended Kalman Filter (EKF), and Unscented Kalman Filter (UKF) for state estimation.

- [Particle Filters](./notes/particle_filter.html) \
  Monte Carlo Localization, resampling strategies, and handling non-Gaussian noise.

- [Sensor Fusion](./notes/sensor_fusion.html) \
  Integrating data from GNSS, IMU, Lidar, and cameras.

- [GNSS & RTK](./notes/gnss.html) \
  GPS fundamentals, error sources, and real-time kinematic corrections.

### Mapping & SLAM

- [SLAM Overview](./notes/slam_overview.html) \
  Simultaneous Localization and Mapping concepts and problem formulation.

- [Lidar-based SLAM](./notes/lidar_slam.html) \
  Scan matching, ICP, and popular frameworks (e.g., Cartographer, LOAM).

- [Visual SLAM](./notes/visual_slam.html) \
  Feature-based vs. direct methods, monocular/stereo/visual-inertial SLAM.

- [Occupancy Grids & Mapping](./notes/occupancy_mapping.html) \
  Grid-based world representations and map updates.

### Motion Planning

- [Path Planning Overview](./notes/path_planning.html) \
  Graph-based, sampling-based, and optimization-based methods.

- [A* and D* Variants](./notes/a_star.html) \
  Heuristic search methods for grid-based navigation.

- [RRT and RRT*](./notes/rrt.html) \
  Sampling-based planning for high-dimensional spaces.

- [Model Predictive Path Planning](./notes/mpc_planning.html) \
  Using MPC for safe, dynamic motion planning.

- [Trajectory Tracking](./notes/trajectory_tracking.html) \
  Controllers and feedback laws for following planned paths.

## 🚗 Domain Specific Navigation

- [Autonomous Cars](./domain_specific_nav/cars.html) \
  Lane detection, GPS+IMU fusion, and road network navigation.

- [Aerial Robots](./domain_specific_nav/drones.html) \
  Visual-inertial odometry and global navigation strategies.

## 🛠️ Tools and Frameworks

- [ROS Navigation Stack](./tools/ros_nav_stack.html) \
  Overview of `move_base`, planners, costmaps, and configuration.

- [Simulation Environments](./tools/simulation.html) \
  Gazebo, Webots, and CARLA for navigation testing.

---

*Made using [Jekyll](https://jekyllrb.com/)*

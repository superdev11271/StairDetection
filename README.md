# Staircase Autonomy

This repository contains the ROS 2 implementation for a staircase perception and modeling pipeline. The algorithms in this work are designed to enable autonomous robots to reliably detect, model, and traverse staircases using 3D point cloud data.

---

## Overview

This package implements and merges the work from two research papers. The core functionality includes:

1.  [**Fast Staircase Detection and Multi-Robot Merging (ICRA 2023):**](https://www.prassi.me/fast-staircase-detection) A computationally efficient method for detecting staircases from 3D point clouds. It relies on extracting line segments from a pre-processed point cloud and applying a set of geometric constraints to identify potential stair structures. This approach also includes a method for merging detections from multiple viewpoints or robots to create a more complete estimate of the staircase.

2.  [**Bayesian Modeling and Inference for Cluttered Staircases (RA-L 2025):**](https://www.prassi.me/bayesian-staircase-estimation) A probabilistic framework that improves staircase estimation, especially in cluttered scenes where stairs might be partially obscured. This method uses a Bayesian model to combine measurements (detections) using an Extended Kalman Filter. We can then segment points on stair surfaces usin the estimated staircase location.

---
## Quick Start

### Installation

This package has been fully tested with ROS2 Humble. To build the packages, clone this repository into your ROS 2 workspace and build it using colcon:

```
cd your_ros2_ws/src
git clone https://github.com/prassi07/staircase_autonomy.git
cd ..
colcon build --symlink-install --packages-up-to staircase_perception
source install/setup.bash
```

### Staircase Estimation (Single and Multi-Robot)
You can launch the robot staircase estimation node as shown below. The node takes in Odometry and Registered Point Clouds from SLAM as input and detects, models and estimates staircases. The estimated staircases can be visualized in RViz2. 

```
ros2 launch staircase_perception staircase_estimation_robot_nodes.launch.py robot_namespace:=/robot_A launch_marker_publisher:=true
```

If you have a multi-robot setup, you can launch the above node on each of the robots and aggregate estimates. On an off-robot machine with communications to the robot, you can run the client node that can merge esimates from multiple robots.

```
ros2 launch staircase_perception staircase_client_nodes.launch.py 
```

---

For detailed instructions on system architecture, configuration, and usage, please refer to the documentation in the `docs` folder. 

* `docs/system_overview.md` provides a high level overview of all the ROS2 packages and the data flow
* `docs/ros2_nodes/**.md` provides a detailed overview of the inputs/outputs and parameters to different ROS nodes that are present in the pacakge

---
## Citations

This work is based on the following publications. If you use this software in your research, please consider citing our papers.

```bibtex
@inproceedings{sriganesh2023fast,
  title={Fast staircase detection and estimation using 3d point clouds with multi-detection merging for heterogeneous robots},
  author={Sriganesh, Prasanna and Bagree, Namya and Vundurthy, Bhaskar and Travers, Matthew},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={9253--9259},
  year={2023},
  organization={IEEE}
}

@article{sriganesh2025bayesian,
  title={A Bayesian Modeling Framework for Estimation and Ground Segmentation of Cluttered Staircases},
  author={Sriganesh, Prasanna and Shirose, Burhanuddin and Travers, Matthew},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  publisher={IEEE}
}
```
---
## LICENSE

This software is released under the BSD 3-Clause License. See the `LICENSE` file in the root directory for the full license text.

---
## Acknowledgments
* I would like to extend my sincere gratitude to Dr. Matthew Travers and Dr. Bhaskar Vundurthy for their invaluable advice and support throughout this project.

* Thanks to Namya Bagree and Burhanuddin Shirose for their contributions during the development of this project.

* This project uses modified code from the **[laser_line_extraction](https://github.com/kam3k/laser_line_extraction)** repository for its line segmentation features.
  * Derived Files:
      * `staircase_perception/src/utils/line_extraction/*`
      * `staircase_perception/include/staircase_perception/utils/line_extraction/*`
  * Original Copyright: `Copyright (c) 2014, Marc Gallant`
  *  Original License: `BSD 3-Clause` (The full license text is included in the derived source files)

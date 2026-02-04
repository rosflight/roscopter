[![ROS2 CI](https://github.com/rosflight/roscopter/actions/workflows/ros2-ci.yml/badge.svg?branch=main)](https://github.com/rosflight/roscopter/actions/workflows/ros2-ci.yml)

<p align="center">
  <img src="wiki_resources/ROScopterLogo.png" alt="ROScopter logo" width=500 />
</p>

<div align="center">
    <video autoplay loop src="https://github.com/user-attachments/assets/9037061c-fc99-4d24-8903-e6fc53d9af00">
</div>

ROScopter is a basic multirotor autopilot build around ROS2 for use with the ROSflight autopilot.
It is a continuation of the original [ROScopter](https://github.com/byu-magicc/roscopter) project.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

## Installation, Descriptions, Usage
For more information, see the [ROSflight documentation](https://docs.rosflight.org), which contains installation, general descriptions, and tutorials for how to use ROScopter. 

## Quick start (for ROS2 users)

For more detailed instructions, see the documentation link above.

1. Clone this repo into a ROS2 workspace
2. `colcon build`
3. Launch autopilot stack with:
  ```bash
  ros2 launch roscopter roscopter.launch.py
  ```
  or if in sim:
  ```bash
  ros2 launch roscopter_sim sim.launch.py
  ```

## Acknowledgements
A special thanks is due to the developers of ROScopter v1.0.
Their work allowed for the project in its current form.

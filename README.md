# ras_choreographies
Repository for autonomous UAV choreographies at RAS.

## Branches.
The main branch contains code designed to work with [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) with ROS2 installed, whereas the ros1 branch works with the previous [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) project, which is no longer maintained.

## Usage.
This repository is made to be used with [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) for the Crazyflies UAVs by [Bitcraze](https://github.com/bitcraze) as a ROS2 package.

- 1. Follow the instructions in [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) for installing their packages in your system, the supported OS and ROS distros are constantly being updated, so you may be checking the docs once in a while.
- 2. Clone this repo by running `git clone https://github.com/santiagorg2401/ras_choreographies.git` inside the `~/path_to/ros2_ws/src/crazyswarm2` folder.
- 3. Build the package using ```colcon build --packages-select ras_choreographies```, for convenience, use symlink `colcon build --symlink-install`.
- 4. To run any code you may follow the same commands as in [the Crazyswarm2 docs](https://imrclab.github.io/crazyswarm2/usage.html) changing the package name from `crazyflie_examples` to `ras_choreographies`
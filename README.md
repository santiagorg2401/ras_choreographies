# ras_choreographies
Repository for autonomous UAV choreographies at RAS.

## Usage.
This repository is made to be used with [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) for the Crazyflies UAVs by [Bitcraze](https://github.com/bitcraze).

- 1. Follow the instructions in [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) for installing their packages in your system, the supported OS and ROS distros are constantly being updated, so you may be checking the docs once in a while.
- 2. Clone this repo by running ```git clone https://github.com/santiagorg2401/ras_choreographies.git``` inside the ```~/path_to/crazyswarm/scripts``` folder.
- 3. To run any code you may follow the same commands as in [Switching between simulation and real hardware](https://crazyswarm.readthedocs.io/en/latest/api.html#switching-between-simulation-and-real-hardware), however, several programs require to modify [crazyswarm_py.py](https://github.com/USC-ACTLab/crazyswarm/blob/4d6ca47b085227fbc893479894001d1c7ceab5cc/ros_ws/src/crazyswarm/scripts/pycrazyswarm/crazyswarm_py.py#L48) as well, adding ```self.simTrue = args.sim``` after line 48.

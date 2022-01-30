#!/usr/bin/env python3
import sys

# Change your user.
sys.path.append('/home/santiagorg2401/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')

import numpy as np
from pycrazyswarm import *
import uav_trajectory

# Example code of using the Crazyflie's trajectories functions. 

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 2
    channel: 80
    initialPosition: [1.5, 1.3, 0.0]
    type: default
"""

#Collision avoidance (between Crazyflies) parameters.
xy_radius = 0.25
radii = xy_radius * np.array([1.0, 1.0, 3.0])

class sender():
    def __init__(self):
        # Constructor of the class sender.
        # Create the needed objects and initialize a ROS Node.
        self.swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
        self.allcfs = self.swarm.allcfs
        self.cfs = self.allcfs.crazyflies
        self.timeHelper = self.swarm.timeHelper

        # Set solid color effect.
        for cf in self.allcfs.crazyflies:
            cf.setParam("ring/effect", 7)
        
        # Enable colission avoidance between all Crazyflies.
        for i, cf in enumerate(self.cfs):
            others = self.cfs[:i] + self.cfs[(i+1):]
            cf.enableCollisionAvoidance(others, radii)

            self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def publisher(self):

        # Get the trajectory.
        traj = uav_trajectory.Trajectory()
        traj.loadcsv("traj.csv")

        TRIALS = 1
        TIMESCALE = 3.0

        for i in range(TRIALS):
            for cf in self.allcfs.crazyflies:
                cf.uploadTrajectory(0, 0, traj)

            # Confirmation.
            print("Press any button to start with the sequence.")
            self.swarm.input.waitUntilButtonPressed()
            self.allcfs.takeoff(targetHeight=0.3, duration = 3)
            self.timeHelper.sleep(3.5)

            # Set color.
            for cf in self.allcfs.crazyflies:
                cf.setLEDColor(1, 0, 0)

            self.allcfs.startTrajectory(0, timescale=TIMESCALE)
            self.timeHelper.sleep(traj.duration * TIMESCALE + 2.0)

            self.allcfs.land(targetHeight=0.02, duration=3.0)
            self.timeHelper.sleep(3.0)

if __name__ == "__main__":
    sen = sender()
    sen.publisher()
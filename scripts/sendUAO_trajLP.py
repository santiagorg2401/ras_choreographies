#!/usr/bin/env python3
import sys
import os

sys.path.append('/home/'+ os.getlogin() + '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')

import numpy as np
from pycrazyswarm import *
import uav_trajectory

# This code performs a UAO choreography for light-painting use, it can be executed with smooth trajectories using "publisherTraj".

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 2
    channel: 80
    initialPosition: [1.0, 0.68, 0.0]
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

    def publisherTraj(self):
        # Use uav_trajectories to execute a smooth 8th order polynomical trajectory.

        # Get the U trajectory.
        trajU = uav_trajectory.Trajectory()
        trajU.loadcsv("trajectories/traj_U.csv")

        # Get the A trajectory.
        trajA = uav_trajectory.Trajectory()
        trajA.loadcsv("trajectories/traj_A.csv")

        # Get the O trajectory.
        trajO = uav_trajectory.Trajectory()
        trajO.loadcsv("trajectories/traj_O.csv")

        trajUAO = uav_trajectory.Trajectory()
        trajUAO.loadcsv("trajectories/traj_UAO.csv")

        TRIALS = 1
        TIMESCALE = 1.5

        for i in range(TRIALS):
            for cf in self.allcfs.crazyflies:
                cf.uploadTrajectory(0, 0, trajUAO)

        # Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        # Take off.
        self.allcfs.takeoff(targetHeight=1.54, duration = 7.0)
        self.timeHelper.sleep(5.0)

        # Set color.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(1, 0, 0)
        
        # Start trajectory.
        self.allcfs.startTrajectory(0, timescale=TIMESCALE)
        self.timeHelper.sleep(trajU.duration * TIMESCALE + 5.0)

        # Set color.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(1, 1, 1)

        self.timeHelper.sleep(trajA.duration * TIMESCALE + 5.0)
        
        # Set color.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(1, 0.5, 1)

        self.timeHelper.sleep(trajO.duration * TIMESCALE + 5.0)
        
        # Land.
        self.allcfs.land(targetHeight=0.02, duration=5.0)
        self.timeHelper.sleep(7.0)
    

if __name__ == "__main__":
    sen = sender()
    sen.publisherTraj()

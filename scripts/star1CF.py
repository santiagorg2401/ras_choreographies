#!/usr/bin/env python3
import sys

# Change your user.
sys.path.append('/home/santiagorg2401/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')

import numpy as np
from pycrazyswarm import *
from waypointsPublisher import waypointsPublisher
import uav_trajectory

# This code performs a star choreography for light-painting use, it can be executed with smooth trajectories using "publisherTraj" or linear ones using "publisherLin".

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 2
    channel: 80
    initialPosition: [1.0, 0.63, 0.0]
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
        # Get the trajectory.
        traj = uav_trajectory.Trajectory()
        traj.loadcsv("traj_star.csv")

        TRIALS = 1
        TIMESCALE = 4.0

        for i in range(TRIALS):
            for cf in self.allcfs.crazyflies:
                cf.uploadTrajectory(0, 0, traj)

        # Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        # Take off.
        self.allcfs.takeoff(targetHeight=0.49, duration = 3)
        self.timeHelper.sleep(3.5)

        # Set color.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(1, 0, 0)
        
        # Start trajectory.
        self.allcfs.startTrajectory(0, timescale=TIMESCALE)
        self.timeHelper.sleep(traj.duration * TIMESCALE + 2.0)

        # Land.
        self.allcfs.land(targetHeight=0.02, duration=3.0)
        self.timeHelper.sleep(3.0)
    
    def publisherLin(self):
        # Use waypointsPublisher.py to get the waypoints and execute linear trajectories.
        # Adquire the desired coordinates list from waypointsPublisher.py
        wpp = waypointsPublisher()
        star = wpp.star()

        # Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()
        
        # Take off.
        self.allcfs.takeoff(targetHeight=0.3, duration=3.0)
        self.timeHelper.sleep(3.0)

        # Set color.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(1, 0, 1)

        # Perform
        for i in range(len(star)):
            data = star[i]
            pos = data[0:3]

            self.cfs[0].goTo(pos, data[3], data[4])
            self.timeHelper.sleep(data[4] + 1.0)

        # Land
        self.allcfs.land(targetHeight=0.02, duration=3.0)
        self.timeHelper.sleep(3.0)

if __name__ == "__main__":
    sen = sender()

    option = int(input("Do you want to use smooth polynomical trajectories? 1 for yes, else not: "))

    if option == 1:
        sen.publisherTraj()
    else:
        sen.publisherLin()
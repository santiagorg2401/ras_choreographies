#!/usr/bin/env python3
import numpy as np
import sys
import os

sys.path.append('/home/' + os.getlogin() +
                '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')
os.chdir("/home/" + os.getlogin() +
         "/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/")

from pycrazyswarm import *

# Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 1
    channel: 80
    initialPosition: [0.5, 0.5, 0.0]
    type: default
  - id: 2
    channel: 80
    initialPosition: [1.5, 0.5, 0.0]
    type: default
  - id: 3
    channel: 80
    initialPosition: [1.0, 1.0, 0.0]
    type: default
  - id: 4
    channel: 80
    initialPosition: [0.5, 1.5, 0.0]
    type: default
  - id: 5
    channel: 80
    initialPosition: [1.5, 1.5, 0.0]
    type: default
"""


class sender():
    def __init__(self, speed):
        # Constructor of the class sender.
        # Create the needed objects and initialize a ROS Node.
        # I nstance from the class Crazyswarm in pycrazyswarm.crazyswarm
        self.swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
        # Instance from the class CrazyflieServer in pycrazyswarm.crazyflie
        self.allcfs = self.swarm.allcfs
        # List of objects  from the class Crazyflie in pycrazyswarm.crazyflie, it has as many objects as declared in crazyflies_yaml, in the same order starting from 0.
        self.cfs = self.allcfs.crazyflies
        # Instance from the class timeHelper in pycrazyswarm.crazyflie
        self.timeHelper = self.swarm.timeHelper

        self.speed = speed

        # Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
        for cf in self.allcfs.crazyflies:
            cf.setParam("ring/effect", 7)
            cf.setParam("ring/headlightEnable", 0)
            cf.setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def publisher(self):
        # Set up mask.
        self.cfs[0].setGroupMask(0)
        self.cfs[1].setGroupMask(0)
        self.cfs[3].setGroupMask(0)
        self.cfs[4].setGroupMask(0)
        
        # Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        # Take off.
        self.cfs[2].takeoff(targetHeight=1.5, duration=1.5/self.speed)
        self.timeHelper.sleep(1.5/self.speed + 1.0)

        self.allcfs.takeoff(targetHeight=1.0, duration=1.0 /
                            self.speed, groupMask=0)
                            
        self.timeHelper.sleep(1.0/self.speed*1.5)

        # Change colour.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(255/255.0, 0/255.0, 0/255.0)
            cf.setParam("ring/headlightEnable", 1)

        # Confirmation.
        print("Press any button to finish the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        # Land.
        self.allcfs.land(targetHeight=0.02, duration=1.5/self.speed)
        self.timeHelper.sleep(1.5/self.speed*1.5)

    def calculateTime(self, speed, initialPos, finalPos):
        distance = abs(np.linalg.norm(finalPos - initialPos))
        Time = distance/speed

        return Time


if __name__ == "__main__":
    speed = 1.0

    sen = sender(speed=speed)
    sen.publisher()

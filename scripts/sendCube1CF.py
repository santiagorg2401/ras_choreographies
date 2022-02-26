#!/usr/bin/env python3
import sys
import os

sys.path.append('/home/'+ os.getlogin() + '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')

import numpy as np
from pycrazyswarm import *
from waypointsPublisher import waypointsPublisher

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 2
    channel: 80
    initialPosition: [1.5, 1.5, 0.0]
    type: default
"""

#Collision avoidance (between Crazyflies) parameters.
xy_radius = 0.25
radii = xy_radius * np.array([1.0, 1.0, 3.0])

class sender():
    def __init__(self):
      #Constructor of the class sender.
      #Create the needed objects and initialize a ROS Node.
      self.swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)  #Instance from the class Crazyswarm in pycrazyswarm.crazyswarm
      self.allcfs = self.swarm.allcfs                           #Instance from the class CrazyflieServer in pycrazyswarm.crazyflie
      self.cfs = self.allcfs.crazyflies                         #List of objects  from the class Crazyflie in pycrazyswarm.crazyflie, it has as many objects as declared in crazyflies_yaml, in the same order starting from 0.
      self.timeHelper = self.swarm.timeHelper                   #Instance from the class timeHelper in pycrazyswarm.crazyflie 

      #Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
      for cf in self.allcfs.crazyflies:
        cf.setParam("ring/effect", 7)

      #Enable collision avoidance between all Crazyflies.
      for i, cf in enumerate(self.cfs):
        others = self.cfs[:i] + self.cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)
        
        self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def publisher(self):
        #Adquire the desired coordinates list from waypointsPublisher.
        wpp = waypointsPublisher()
        cube = wpp.cube()

        #Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        #Take off.
        self.allcfs.takeoff(targetHeight=0.3, duration= 3.0)
        self.timeHelper.sleep(3.0)

        #Perform.
        for i in range(len(cube)):
            data = cube[i]
            pos = data[0:3]

            self.cfs[0].goTo(pos, data[3] , data[4])
            self.timeHelper.sleep(data[4] + 1.0)
        
        #Land.
        self.allcfs.land(targetHeight=0.02, duration=3.0)
        self.timeHelper.sleep(3.0)

if __name__ == "__main__":
    sen = sender()
    sen.publisher()
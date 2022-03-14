#!/usr/bin/env python3
import numpy as np
import sys
import os

sys.path.append('/home/'+ os.getlogin() + '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')
os.chdir("/home/" + os.getlogin() + "/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/")

from pycrazyswarm import *

class sender():
    def __init__(self, path, speed):
      # Constructor of the class sender.
      # Create the needed objects and initialize a ROS Node.
      self.swarm = Crazyswarm()  #I nstance from the class Crazyswarm in pycrazyswarm.crazyswarm
      self.allcfs = self.swarm.allcfs                           # Instance from the class CrazyflieServer in pycrazyswarm.crazyflie
      self.cfs = self.allcfs.crazyflies                         # List of objects  from the class Crazyflie in pycrazyswarm.crazyflie, it has as many objects as declared in crazyflies_yaml, in the same order starting from 0.
      self.timeHelper = self.swarm.timeHelper                   # Instance from the class timeHelper in pycrazyswarm.crazyflie

      self.path = path
      self.speed = speed

      # Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
      for cf in self.allcfs.crazyflies:
        cf.setParam("ring/effect", 7)
        cf.setParam("ring/headlightEnable", 0)
        cf.setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def publisher(self):
      # Set up parameters.
      data = np.genfromtxt(self.path, delimiter=',')
      
      takeOffHeight = data[0][2]
      initialPos = data[0] - np.array([0, 0, takeOffHeight])

      # Confirmation.
      print("Press any button to start with the sequence.")
      self.swarm.input.waitUntilButtonPressed()

      # Take off.
      self.allcfs.takeoff(targetHeight=takeOffHeight, duration=takeOffHeight/self.speed)
      self.timeHelper.sleep(takeOffHeight/self.speed + 0.1)
      
      # Change colour.
      for cf in self.allcfs.crazyflies:
        cf.setLEDColor(255/255.0, 0/255.0, 0/255.0)
        cf.setParam("ring/headlightEnable", 1)

      for i in range(0, len(data)):
          for cf in self.allcfs.crazyflies:
            if i == 0:
              Time = self.calculateTime(self.speed, initialPos, data[i])
            else:
              Time = self.calculateTime(self.speed, data[i - 1], data[i])

            cf.goTo(data[i], 0, Time)
            self.timeHelper.sleep(Time + 0.1)

      #Land.
      self.allcfs.land(targetHeight=0.02, duration = (data[-1][2] - 0.02)/speed)
      self.timeHelper.sleep((data[-1][2] - 0.02)/speed + 1.0)

    def calculateTime(self, speed, initialPos, finalPos):
      distance = abs(np.linalg.norm(finalPos - initialPos))
      Time = distance/speed

      return Time

if __name__ == "__main__":
  path = 'ras_choreographies/scripts/waypoints/boat2.csv'
  speed = 1.0

  sen = sender(path=path, speed=speed)
  sen.publisher()
#!/usr/bin/env python3
import numpy as np
import sys
import os
import rospy

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
      self.led(0)

    def publisher(self):
      # Set up parameters.
      data = np.genfromtxt(self.path, delimiter=',')
      
      takeOffHeight = data[0][2]
      initialPos = data[:, 0:3][0] - np.array([0, 0, takeOffHeight])

      # Confirmation.
      print("Press any button to start with the sequence.")
      self.swarm.input.waitUntilButtonPressed()

      # Take off.
      self.allcfs.takeoff(targetHeight=takeOffHeight, duration=takeOffHeight/self.speed)
      self.timeHelper.sleep(takeOffHeight/self.speed*1.2)

      self.led(1)
      led = 1

      for i in range(0, len(data)):
        # try:
        #   if data[i][3] == 0 & led == 1:
        #     self.led(0)
        #     led = 0
        #   elif data[i][3] == 1 & led == 0:
        #     self.led(1)
        #     led = 1
        # except:
        #   pass

          for cf in self.allcfs.crazyflies:
            if i == 0:
              Time = self.calculateTime(self.speed, initialPos, data[:, 0:3][i])
            else:
              Time = self.calculateTime(self.speed, data[:, 0:3][i - 1], data[:, 0:3][i])

            cf.goTo(data[:, 0:3][i], 0, Time)
            self.timeHelper.sleep(Time*1.5)

      #Land.
      self.allcfs.land(targetHeight=0.02, duration = (data[-1][2] - 0.02)/speed)
      self.timeHelper.sleep((data[-1][2] - 0.02)/speed*1.2)

    def calculateTime(self, speed, initialPos, finalPos):
      distance = abs(np.linalg.norm(finalPos - initialPos))
      Time = distance/speed

      return Time

    def led(self, enable):
      # Change colour.
      for cf in self.allcfs.crazyflies:
        cf.setLEDColor(255*enable/255.0, 0/255.0, 0/255.0)
        cf.setParam("ring/headlightEnable", enable)

if __name__ == "__main__":
  args = rospy.myargv(argv=sys.argv)
  file_path = args[1]
  speed = 0.6

  sen = sender(path=file_path, speed=speed)
  sen.publisher()
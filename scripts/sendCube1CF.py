#!/usr/bin/env python3
import sys

# Change your user.
sys.path.append('/home/santiagorg2401/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')

import numpy as np
from pycrazyswarm import *
from waypointsPublisher import waypointsPublisher

# Add 'self.simTrue = args.sim' on crazyswarm_py.py after line 48.
# This code requires enabled logging and logging variables health.motorPass & pm.vbat.

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 1
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
      
      #Global variables.
      self.idM = np.array([])       #Crazyflie ID matrix.
      self.batVol = np.array([])    #Battery voltage matrix.
      self.motPas = np.array([])    #Propeller test result.

      #Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
      for cf in self.allcfs.crazyflies:
        cf.setParam("ring/effect", 7)

      #Enable collision avoidance between all Crazyflies.
      for i, cf in enumerate(self.cfs):
        others = self.cfs[:i] + self.cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)

      for i in range(len(self.cfs)):
        if self.swarm.simTrue:
          pass
        else:
          #Import only if physical robots are being used.
          import rospy
          from crazyswarm.msg import GenericLogData as gld

          attrs = dir(self.cfs[i])
          id = getattr(self.cfs[i], attrs[44])
          self.idM = np.append(self.idM, [id], axis=0)

          #Subscribe to i topics, where the needed data are being published.
          self.log1 = rospy.Subscriber("/cf"+ str(id) + "/log1", gld, self.adquireLog1DataCallback)
        
        self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def adquireLog1DataCallback(self, log1Data):
      #Callback function, it extracts the data from the /cfi/log1 topics.
      self.motPas = log1Data.values[0]    #LOG variable health.motorPass
      self.batVol = log1Data.values[1]    #LOG variable pm.vbat

    def preLaunchSequence(self):
      #This method performs a propeller test and checks battery voltage on all listed Crazyflies.
      for i in range(len(self.cfs)):
        print("\nInitializing prelaunch sequence.")

        self.cfs[i].setParam("ring/effect", 6)
        self.cfs[i].setParam("health/startPropTest", 1)
        self.timeHelper.sleep(5.0)
        self.cfs[i].setParam("ring/effect", 7)

        if self.motPas != 15 or self.batVol <= 3.7:
          self.cfs[i].setLEDColor(255/255.0, 0/255.0, 0/255.0)
          print("\nCrazyflie #" + str(self.idM[i]) + " cannot fly.")
          ready = 0

        else:
          print("\nCrazyflie #" + str(self.idM[i]) + " is ready to fly.")
          self.cfs[i].setLEDColor(0/255.0, 255/255.0, 0/255.0)
          ready = 1

        self.cfs[i].setParam("health/startPropTest", 0)

        if self.motPas == 15:
          msg = "succesful."
        else:
          msg = "failure."
        
        print("Propeller test: " + msg)
        print("Battery voltage: " + str(self.batVol) + "V.")

      return ready

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
            self.timeHelper.sleep(3.0)
        
        #Land.
        self.allcfs.land(targetHeight=0.02, duration=3.0)
        self.timeHelper.sleep(3.0)

if __name__ == "__main__":
    sen = sender()

    if sen.swarm.simTrue:
      sen.publisher()
    else:
      if sen.preLaunchSequence()== 1:
        sen.publisher()
      else:
        print("Check the crazyflies.")
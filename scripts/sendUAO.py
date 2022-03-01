#!/usr/bin/env python3
import sys
import os

sys.path.append('/home/'+ os.getlogin() + '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')
os.chdir("/home/" + os.getlogin() + "/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/")

from waypointsPublisher import waypointsPublisher
from pycrazyswarm import * 
import numpy as np

# Add 'self.simTrue = args.sim' on crazyswarm.py after line 48.
# This code requires enabled logging and logging variables health.motorPass & pm.vbat.

#Crazyflies list, this is optional, however, it is safer to declare it on code.
crazyflies_yaml = """
crazyflies:
  - id: 1
    channel: 80
    initialPosition: [2.0, 1.1, 0.0]
    type: default
  - id: 2
    channel: 80
    initialPosition: [2.0, 1.8, 0.0]
    type: default
  - id: 3
    channel: 80
    initialPosition: [1.5, 1.1, 0.0]
    type: default
"""
  # - id: 4
  #   channel: 80
  #   initialPosition: [1.5, 1.8, 0.0]
  #   type: default
  # - id: 5
  #   channel: 80
  #   initialPosition: [1.0, 1.2, 0.0]
  #   type: default
  # - id: 6
  #   channel: 80
  #   initialPosition: [1.0, 1.69, 0.0]
  #   type: default
  # - id: 7
  #   channel: 80
  #   initialPosition: [0.5, 1.45, 0.0]
  #   type: default
  # - id: 8
  #   channel: 80
  #   initialPosition: [0.5, 2.5, 0.0]
  #   type: default
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
        
        self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def publisher(self): 
      #Adquire the desired coordinates lists from waypointsPublisher.
      wpp = waypointsPublisher()
      sequences = wpp.UAO()

      #Confirmation.
      print("Press any button to start with the sequence.")
      self.swarm.input.waitUntilButtonPressed()

      #Take off.
      self.allcfs.takeoff(targetHeight=0.3, duration= 3.0)
      self.timeHelper.sleep(4.0)

      #UAO Sequence.
      for j in range(len(sequences)):
        for i in range(len(self.cfs)):
          self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)
          self.cfs[i].setParam("ring/headlightEnable", 1)
          data = sequences[j][i]
          pos = data[0:3]
          self.cfs[i].goTo(pos, data[3], data[4])
          self.timeHelper.sleep(data[4] + 1.0)

          if j == 0:
            if i != 7:
              self.cfs[i].setLEDColor(255/255.0, 0/255.0, 0/255.0)
          elif j == 1:
            if i != 6 and i != 7:
              self.cfs[i].setLEDColor(255/255.0, 140/255.0, 0/255.0)
          elif j == 2:
            self.cfs[i].setLEDColor(255/255.0, 127/255.0, 80/255.0)
      
      #Return.
      for i in range(len(self.cfs)):
        self.cfs[i].setLEDColor(75/255.0, 0/255.0, 130/255.0)

      self.cfs[7].goTo(sequences[0][7][0:3], sequences[0][7][3], sequences[0][7][4] + 3.0)
      self.timeHelper.sleep(sequences[0][7][4] + 3.0)

      for i in range(len(self.cfs)):
        data = sequences[0][i]
        pos = data[0:3]
        self.cfs[i].goTo(pos, data[3], data[4] + 3.0)
        self.timeHelper.sleep(data[4] + 3.0)

      #Land.
      self.allcfs.land(targetHeight=0.002, duration= 7.0)
      self.timeHelper.sleep(9.0)

if __name__ == "__main__":
    sen = sender()
    sen.publisher()
#!/usr/bin/env python3
import sys
import os

sys.path.append('/home/'+ os.getlogin() + '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')
os.chdir("/home/" + os.getlogin() + "/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/")

# TODO: Limitations: This code only works if you test 3 drones at a time and have no more than 4 drones connected.
# Add 'self.simTrue = args.sim' on crazyswarm_py.py after line 48.
# This code requires enabled logging and logging variables health.motorPass & pm.vbat.

import rospy
from crazyswarm.msg import GenericLogData as gld
from pycrazyswarm import * 
import numpy as np

#List the Crazyflies.

class sender():
    def __init__(self):
        #Create the needed objects.
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.cfs = self.allcfs.crazyflies
        self.timeHelper = self.swarm.timeHelper
        
        #Global variables.
        self.idM = np.array([])
        self.batVol = np.array([])
        self.motPas = np.array([])

        #Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
        for cf in self.allcfs.crazyflies:
            cf.setParam("ring/effect", 7)

        for i in range(len(self.cfs)):
            attrs = dir(self.cfs[i])
            id = getattr(self.cfs[i], attrs[44])
            self.idM = np.append(self.idM, [id], axis=0)

            #Subscribe to i topics, where the needed data are being published.
            self.log1 = rospy.Subscriber("/cf"+ str(id) + "/log1", gld, self.adquireLog1DataCallback)
            
            self.cfs[i].setLEDColor(0/255.0, 0/255.0, 0/255.0)

    def adquireLog1DataCallback(self, log1Data):
        self.motPas = log1Data.values[0]    #LOG variable health.motorPass
        self.batVol = log1Data.values[1]    #LOG variable pm.vbat

    def preLaunchSequence(self):
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
                print("Crazyflie #" + str(self.idM[i]) + " is ready to fly.")
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

if __name__ == "__main__":
    sen = sender()
    sen.preLaunchSequence()
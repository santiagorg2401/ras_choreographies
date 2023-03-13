#!/usr/bin/env python3

import numpy as np
from crazyflie_py import Crazyswarm


class sender():
    def __init__(self):
        # Constructor of the class sender.
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.cfs = self.allcfs.crazyflies
        self.timeHelper = self.swarm.timeHelper

        self.allcfs.declare_parameter("path", value="")
        self.allcfs.declare_parameter("speed", vakue=1.0)

        self.path_ = self.allcfs.get_parameter("path").value
        self.speed_ = self.allcfs.get_parameter("speed").value

    def publisher(self):
        # Set up parameters.
        data = np.genfromtxt(self.path_, delimiter=',')

        for i in range(0, len(data)):
            takeOffHeight = data[i][2]
            initialPos = data[:, 0:3][i] - np.array([0, 0, takeOffHeight])

            # Take off.
            self.cfs[i].takeoff(targetHeight=takeOffHeight,
                                duration=takeOffHeight/self.speed_ )
            self.timeHelper.sleep(takeOffHeight/self.speed_ *1.2)

            self.cfs[i].goTo(data[:, 0:3][i], 0, 1)
            self.timeHelper.sleep(1.2)

        # Land.
        self.allcfs.land(targetHeight=0.02, duration=(
            takeOffHeight)/self.speed_ )
        self.timeHelper.sleep((takeOffHeight)/self.speed_ *1.2)

    def calculateTime(self, speed, initialPos, finalPos):
        distance = abs(np.linalg.norm(finalPos - initialPos))
        Time = distance/speed

        return Time

def main():
    try:
        sen = sender()
        sen.publisher()
    except KeyboardInterrupt:
        print("Emergency cancel registred, landing UAVs.")
        sen.allcfs.land(targetHeight=0.02, duration=5)
        sen.timeHelper.sleep(5*1.2)
        sen.allcfs.emergency()


if __name__ == "__main__":
    main()
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
        self.allcfs.declare_parameter("speed", value=1.0)

        self.path_ = self.allcfs.get_parameter("path").value
        self.speed_ = self.allcfs.get_parameter("speed").value

    def publisher(self):
        # Set up parameters.
        path = "/home/santiagorg2401/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/ras_choreographies/scripts/waypoints/SGI_lp_wpts.csv"
        data = np.genfromtxt(path, delimiter=',')

        takeOffHeight = data[0][2]
        initialPos = data[:, 0:3][0] - np.array([0, 0, takeOffHeight])

        # Take off.
        self.allcfs.takeoff(targetHeight=takeOffHeight,
                            duration=takeOffHeight/self.speed_)
        self.timeHelper.sleep(takeOffHeight/self.speed_*1.2)

        for i in range(0, len(data)):
            for cf in self.allcfs.crazyflies:
                if i == 0:
                    Time = self.calculateTime(
                        self.speed_, initialPos, data[:, 0:3][i])
                else:
                    Time = self.calculateTime(
                        self.speed_, data[:, 0:3][i - 1], data[:, 0:3][i])

                cf.goTo(data[:, 0:3][i], 0, Time)
                self.timeHelper.sleep(Time*1.5)

        # Land.
        self.allcfs.land(targetHeight=0.02, duration=(
            data[-1][2] - 0.02)/self.speed_)
        self.timeHelper.sleep((data[-1][2] - 0.02)/self.speed_*1.2)

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

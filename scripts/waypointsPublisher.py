#!/usr/bin/env python3

import math as m
import numpy as np

#The waypointsPublisher class contains lists with the desired coordinates (X,Y,Z) expressed in meters, add as many functions as trajectories,
# for example, if you want to perform a cube, then create a function for that containing the necessary coordinates for that, be sure to return it.
class  waypointsPublisher():
    def _init__(self):
        pass

    def map(self):
        #Defines the map where the Crazyflie(s) will fly.
        Map = np.array((2.89, 2.89, 2.0)) - 0.3
        return Map

    def cube(self):
        #A simple cube performed by one Crazyflie.
        # General form: (X,Y,Z, Yaw, Duration (seconds))
        cube = np.array([(1.5, 1.5, 1.0, 0.0, 3.0),
                    (1.5, 1.0, 1.0, 0.0, 3.0),
                    (2.0, 1.0, 1.0, 0.0, 3.0),
                    (2.0, 1.5, 1.0, 0.0, 3.0),
                    (1.5, 1.5, 1.0, 0.0, 3.0),
                    (1.5, 1.5, 1.5, 0.0, 3.0),
                    (1.5, 1.0, 1.5, 0.0, 3.0),
                    (2.0, 1.0, 1.5, 0.0, 3.0),
                    (2.0, 1.5, 1.5, 0.0, 3.0),
                    (1.5, 1.5, 1.5, 0.0, 3.0),
                    (1.5, 1.5, 1.0, 0.0, 3.0)])
        return cube

    def nothing(self):
        #Literally nothing.
        # General form: (X,Y,Z, Yaw, Duration (seconds))
        nothing = np.array([(0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0)])
        return nothing

    def UAO(self):
        # General form: (X,Y,Z, Yaw, Duration (seconds))
        sequenceU = np.array([
            (2.0, 1.1, 1.5, 0.0, 7.0),      #Cf1
            (2.0, 1.8, 1.5, 0.0, 7.0),      #Cf2
            (1.5, 1.1, 1.25, 0.0, 7.0),     #Cf3
            (1.5, 1.8, 1.25, 0.0, 7.0),     #Cf4
            (1.0, (0.7/2)*m.cos(-(3*m.pi)/4) + 1.445, (0.7/2)*m.sin(-(3*m.pi)/4) + 1.25, 0.0, 5.0),    #Cf5
            (1.0, (0.7/2)*m.cos(-(m.pi)/4) + 1.445, (0.7/2)*m.sin(-(m.pi)/4) + 1.25, 0.0, 5.0),        #Cf6
            (0.5, 1.45, 0.95, 0.0, 7.0),    #Cf7
            (0.5, 2.5, 0.3, 0.0, 3.0),      #Cf8 ###
        ])

        sequenceA = np.array([
            (2.5, 1.45, 1.4, 0.0, 5.0),     #Cf1
            (2.0, 1.45, 1.0, 0.0, 5.0),     #Cf2
            (1.5, 1.2, 1.0, 0.0, 5.0),      #Cf3
            (1.5, 1.7, 1.0, 0.0, 5.0),      #Cf4
            (1.0, 0.95, 0.6, 0.0, 5.0),     #Cf5
            (1.0, 1.95, 0.6, 0.0, 5.0),     #Cf6
            (0.5, 1.09, 0.3, 0.0, 7.0),     #Cf7 ###
            (0.5, 2.5, 0.3, 0.0, 5.0),      #Cf8 ###
        ])

        sequenceO = np.array([
            (2.5, 1.45, 1.5, 0.0, 5.0),     #Cf1
            (2.0, 1.45, 0.5, 0.0, 5.0),     #Cf2
            (1.5, 0.5*m.cos((3*m.pi)/4) + 1.445, 0.5*m.sin((3*m.pi)/4) + 1, 0.0, 5.0),      #Cf3
            (1.5, 0.5*m.cos((m.pi)/4) + 1.445, 0.5*m.sin((m.pi)/4) + 1, 0.0, 5.0),          #Cf4
            (1.0, 0.95, 1.0, 0.0, 5.0),     #Cf5
            (1.0, 1.95, 1.0, 0.0, 5.0),     #Cf6
            (0.5, 0.5*m.cos((-3*m.pi)/4) + 1.445, 0.5*m.sin((-3*m.pi)/4) + 1, 0.0, 6.0),    #Cf7
            (0.5, 0.5*m.cos((-m.pi)/4) + 1.445, 0.5*m.sin((-m.pi)/4) + 1, 0.0, 5.0),        #Cf8
        ])

        return sequenceU, sequenceA, sequenceO
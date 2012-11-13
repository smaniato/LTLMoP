#!/usr/bin/env python
"""
=========================================
iRobotCreateActuator.py - Actuator Handler for iRobotCreate
=========================================
"""

import time
from struct import pack,unpack
from threading import Thread, Event
from numpy import *
from scipy.linalg import norm


class actuatorHandler:
    def __init__(self, proj, shared_data):

        self.iRobotCommunicator = proj.shared_data['robocomm']
        self.pose_handler = proj.h_instance['pose']
        self.drive_handler = proj.h_instance['drive']
        self.proj = proj

               
   

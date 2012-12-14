#!/usr/bin/env python
"""
===============================================
pioneerActuator.py
===============================================
"""

import time, math, sys
import threading
from numpy import *
from numpy.linalg import norm
from socket import *

class actuatorHandler:
    def __init__(self, proj, shared_data):
        try:
            self.host = shared_data['PIONEER_ADDRESS']
        except KeyError:
            print "ERROR: You need to call Pioneer Init handler before using Pioneer Actuator Handler"
            return
            
        sys.path.append(proj.ltlmop_root)
        from lib.handlers.motionControl.__is_inside import is_inside

        self.proj = proj

    def moveFlag(self, flag_port, actuatorVal, initial):
        """
        Move flag to vertical position (enabled) or horizontal position (disabled).

        flag_port (int): Port to send command to (default=11004)
        """
        if initial:
            self.s_flag = socket(AF_INET,SOCK_DGRAM)
            return
        
        self.s_flag.sendto("T" if int(actuatorVal) == 1 else "F", (self.host, flag_port))
        print "(ACT) Flag is now %s!" % str(actuatorVal)
        
    def doExploration(self, explore_port, actuatorVal, initial):
        """
        Explore the extents of the current (non-finalized) room.

        explore_port (int): Port to send command to (default=11024)
        """
        if initial:
            self.s_explore = socket(AF_INET,SOCK_DGRAM)
            return
        
        self.s_explore.sendto("T" if int(actuatorVal) == 1 else "F", (self.host, explore_port))
        print "(ACT) Explore is now %s!" % str(actuatorVal)

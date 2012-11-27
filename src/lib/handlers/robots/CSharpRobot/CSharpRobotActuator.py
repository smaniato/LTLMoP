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
from ltlmopMsg_pb2 import *
from CSharpAckMsg_pb2 import *


class actuatorHandler:
    def __init__(self, proj, shared_data):

        self.CSharpCommunicator = proj.shared_data['robocomm']
        self.pose_handler = proj.h_instance['pose']
        self.drive_handler = proj.h_instance['drive']
        self.proj = proj

    def explore(self,actuatorVal,initial=False):
        """
        tells C# interface that it needs to explore new region
        """
        if initial is False:
            if actuatorVal == True:
                print 'DO EXPLORE :D'
                ltlmop_msg = PythonRequestMsg()
                ltlmop_msg.id=3
                ltlmop_msg.sensor = PythonRequestMsg.NOSENSOR
                #tlmop_msg.actuator = ltlmop_msg.Actuator() # needs to be changed!
                ltlmop_msg.actuator.actuatorType = PythonRequestMsg.EXPLORE
                ltlmop_msg.actuator.status = PythonRequestMsg.REQ_DO
                response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
        

               
   

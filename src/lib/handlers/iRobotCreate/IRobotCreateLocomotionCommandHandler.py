#!/usr/bin/env python
"""
================================================================================
pioneerExampleLocomotionCommand.py - Pioneer Locomotion Command Handler
================================================================================
"""
import socket, sys, time, logging


import lib.handlers.handlerTemplates as handlerTemplates

class IRobotCreateLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data):
        try:
            self.robocomm = shared_data['robocomm']
        except KeyError, ValueError:
            print "(LOCO) ERROR: No RobotCommunicator set to key 'robocomm' in shared data from init."
            exit(-1)

    def sendCommand(self, cmd):
        # For avoiding the create's dead band
        MIN_V = .05
        MIN_W = .07
        
        v, w = cmd[:2]
        
        scaledCount = 0         # For when both require scaling
        vScale, wScale = 1,1
        if v != 0:
            if abs(v) < MIN_V:
                vScale = abs(MIN_V/v) 
                scaledCount += 1
        if w != 0:
            if abs(w) < MIN_W:
                wScale = abs(MIN_W/w)
                scaledCount += 1
                
        if scaledCount < 2:
            scale = max(vScale, wScale)
        else:
            scale = min(vScale, wScale)
        
        v *= scale
        w *= scale
            
        logging.debug("\nVel: {}".format((v, w)))
        
        direction = (v, w)
        self.robocomm.sendDirection(direction)

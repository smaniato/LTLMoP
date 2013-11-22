#!/usr/bin/env python
"""
=================================================
dipolarDrive.py - Dipolar Drive Handler
=================================================

This class is only placed here to comply with the 
LTLMoP structure.
"""

class driveHandler:
    def __init__(self, proj, shared_data):
        """ Initialization method of differential drive handler. """   
        try:
            self.loco = proj.h_instance['locomotionCommand']
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, v, w, theta=0):
        """ Set the linear and angular velocity of the robot. """
        
        self.loco.sendCommand([v,w])


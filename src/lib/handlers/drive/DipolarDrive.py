#!/usr/bin/env python
"""
=================================================
dipolarDrive.py - Dipolar Drive Handler
=================================================

This class is only placed here to comply with the 
LTLMoP structure.
"""
from threading import Thread, Lock
from time import clock, sleep

class driveHandler:
    def __init__(self, proj, shared_data, frequency):
        """ Closed loop handler that continuously reads pose and updates 
        velocity
        
        frequency (float): The rate at which to change velocity (default=20)
        """            
        # Get references to handlers we'll need to communicate with
        self.locoH = proj.h_instance['locomotionCommand']
        self.poseH = proj.h_instance['pose']
        self.motionH = proj.h_instance['motionControl']
        
        # Start the thread
        self.frequency = frequency
        self.running = True
        self.dipoleLock = Lock()
        self.commandThread = Thread(target=self.closedLoop())
        self.commandThread.start()
        
    def closedLoop(self):
        lastCT = clock()
        prevPose = self.poseH.getPose()
        
        while self.running:
            startT = clock()
            
            getControls = self.motionH.dipController.getControls
            with self.dipoleLock:
                poseT = self.poseH.getPose()
                elapsedT = clock() - lastCT
                v, w = getControls(poseT, self.nextDipole, prevPose, elapsedT)
                self.locoH.sendCommand([v, w])
                lastCT = clock()
                prevPose = poseT
                
            timeLeft = (1/self.frequency) - (clock() - startT)
            if timeLeft > 0:
                sleep(timeLeft)            

    def _stop(self):
        print "Dipolar Drive handler quitting..."
        self.running = False
        print "Terminated."

    def setVelocity(self, dipole):
        """ Set the next dipole. """
        self.nextDipole = dipole
        


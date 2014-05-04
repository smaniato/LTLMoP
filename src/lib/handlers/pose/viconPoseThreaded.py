#!/usr/bin/env python
"""
============================================
viconPose.py - Pose Handler for Vicon System
============================================
"""

import sys, time
import numpy as np
import _pyvicon
from threading import Thread, Lock

import logging

class poseHandler:
    def __init__(self, proj, shared_data,host,port,x_VICON_name,y_VICON_name,theta_VICON_name, frequency):
        """
        Pose handler for VICON system

        host (string): The ip address of VICON system (default="10.0.0.102")
        port (int): The port of VICON system (default=800)
        x_VICON_name (string): The name of the stream for x pose of the robot in VICON system (default="SubjectName:SegmentName <t-X>")
        y_VICON_name (string): The name of the stream for y pose of the robot in VICON system (default="SubjectName:SegmentName <t-Y>")
        theta_VICON_name (string): The name of the stream for orintation of the robot in VICON system (default="SubjectName:SegmentName <a-Z>")
        frequency (float): The rate at which to sample from vicon (default=20)
        """
        
        self.host = host    
        self.port = port
        self.x = x_VICON_name
        self.y = y_VICON_name
        self.theta = theta_VICON_name

        self.s = _pyvicon.ViconStreamer()
        self.s.connect(self.host,self.port)
        
#         logging.debug("{}, {}, {}".format(self.x, self.y, self.theta))
        self.s.selectStreams(["Time", self.x, self.y, self.theta])

        self.s.startStreams()

        # Wait for first data to come in
        while self.s.getData() is None: pass
        
        self.data = (0, 0, 0, 0)
        
        # Initialize thread to poll at desired speed
        self.frequency = frequency
        self.running = True
        self.dataLock = Lock()
        self.dataThread = Thread(target=self.getStreamData)
        self.dataThread.start()
        
    def getStreamData(self):
        while self.running:
            timeStart = time.clock()
            
            (t, x, y, o) = self.s.getData()
            data = (t/100, x/1000, y/1000, o)
            with self.dataLock:
                self.data = data
                
#             logging.debug("Data: {}".format(data))
            
            timeLeft = (1/self.frequency) - (time.clock() - timeStart)
            if timeLeft > 0:
                time.sleep(timeLeft)
    
    def _stop(self):
        print "Vicon pose handler quitting..."
        self.running = False
        self.s.stopStreams()
        print "Terminated."
        
    def getPose(self, cached=False):
        with self.dataLock:
            return np.array(self.data[1:])


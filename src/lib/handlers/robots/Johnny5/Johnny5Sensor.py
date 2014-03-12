#!/usr/bin/env python
"""
=========================================================
Johnny5Sensor.py - Johnny 5 Robot Sensor Handler
=========================================================
"""
import sys, os
import math

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import handlers.pose._pyvicon as _pyvicon

class Johnny5SensorHandler:
    def __init__(self, proj, shared_data):
        """
        Johnny 5 Robot Sensor handler
        """

        self.johnny5Serial = shared_data["Johnny5Serial"]
        self.pose_handler = proj.h_instance['pose']

    ###################################
    ### Available sensor functions: ###
    ###################################
    def findPoint(self, initial=False):

        if initial:
            print "Connecting to Vicon server..."
            self.viconServer = _pyvicon.ViconStreamer()
            self.viconServer.connect("10.0.0.102", 800)
            
            model_name = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01"
            self.viconServer.selectStreams(["Time"] + ["{} <{}>".format(model_name, s) for s in ("t-X", "t-Y")])
            self.viconServer.startStreams()
            
            # Wait for first data to come in
            while self.viconServer.getData() is None:
                pass
        else:
            (t, x, y) = self.viconServer.getData()
            (t, x, y) = [t/100, x/1000, y/1000]
            
            # Find our current configuration
            pose = self.pose_handler.getPose()
            
            range = 0.7
            # Return true if robot is within range of helmet
            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range

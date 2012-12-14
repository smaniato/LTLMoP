#!/usr/bin/env python
from socket import *
from struct import *
"""
==================================================================
skeletonLocomotionCommand.py - Skeleton Locomotion Command Handler
==================================================================
"""

class locomotionCommandHandler:

    def __init__(self, proj, shared_data):
        # TODO: Initialize any network connections that might be necessary.
        #
        # This is only called once at the beginning of execution.
        #
        # Hostnames/port numbers can be loaded as proj.robot_data['XXXX_PORT'][0]
        # if you stick them in the robot configuration file
        #
        # Note that if the same network socket needs to be shared with another handlers,
        # you should initialize it in an initializationHandler and stick a reference
        # to the socket in shared_data.
        host = "10.0.0.96"
        port = 11000
        self.buf = 1024
        self.iii = 0
        self.addr = (host,port)
        self.s = socket(AF_INET,SOCK_DGRAM)
        #pass

    def sendCommand(self, cmd):
        v, w = cmd
        wp = w
        iiii = self.iii
        msg = pack('ddd',iiii,v,wp)

        #msg = pack('dd',v,wp)
        self.iii = self.iii + 1

        #print "Sending..."
        self.s.sendto(msg,self.addr)
        #print "v: %f , w: %f" % (v, w)
        #data,self.addr = self.s.recvfrom(self.buf)
        #s    print "Read data '%s' from %s!" % (data, self.addr)

#!/usr/bin/env python
"""
==================================================================
SegwayLocomotionCommand.py - Segway Locomotion Command Handler
==================================================================

Send velocity commands to the Segway robot
"""

import LtlmopCsharpMessages_pb2 as Message
import logging 

class SegwayLocomotionCommandHandler:
    def __init__(self, proj, shared_data):
        
        # Connect to the server
        MessageStreamClass = shared_data["MessageStreamClass"]
        self.ip = shared_data["SegwayIP"]
        self.port = shared_data["SegwayPort"]
        
        logging.debug("Ip: {}, Port: {}".format(self.ip, self.port))
        
        self.mStream = MessageStreamClass(self.ip, self.port)
        
        # Request that this connection be used for velocity commands
        request = Message.ServiceRequest()
        request.type = Message.ServiceRequest.VELOCITY_CONTROL
        self.mStream.sendMessage(request.SerializeToString())

    def sendCommand(self, cmd):
        """ Send velocity commands to the Segway
        """
        velMsg = Message.VelocityMessage()
        velMsg.linear = cmd[0]
        velMsg.angular = cmd[1]
        self.mStream.sendMessage(velMsg.SerializeToString())
        
#         logging.debug("{}, {}".format(cmd[0], cmd[1]))
        

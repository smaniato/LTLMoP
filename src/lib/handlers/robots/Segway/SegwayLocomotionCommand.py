#!/usr/bin/env python
"""
==================================================================
SegwayLocomotionCommand.py - Segway Locomotion Command Handler
==================================================================

Send velocity commands to the Segway robot
"""

import socket
import LtlmopCsharpMessages_pb2 as Message

class SegwayLocomotionCommandHandler:
    def __init__(self, proj, shared_data):
        
        # Connect to the server
        self.ip = shared_data["SegwayIP"]
        self.port = shared_data["SegwayPort"]
        self.segwaySocket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.segwaySocket.connect((self.ip, self.port))
        
        # Request that this connection be used for velocity commands
        request = Message.ServiceRequest()
        request.type = Message.ServiceRequest.VELOCITY_CONTROL
        self.segwaySocket.sendall(request.SerializeToString())

    def sendCommand(self, cmd):
        """ Send velocity commands to the Segway
        """
        velMsg = Message.VelocityMessage()
        velMsg.linear = cmd[0]
        velMsg.angular = cmd[0]
        self.segwaySocket.sendall(velMsg.SerializeToString())
        

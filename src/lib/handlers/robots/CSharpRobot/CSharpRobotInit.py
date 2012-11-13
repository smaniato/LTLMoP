#!/usr/bin/env python
"""
=================================================
CSharpRobotInit.py - iRobotCreate Initialization Handler
=================================================
"""

import time, math
from socket import *
from struct import pack,unpack
from threading import Thread, Lock, Event
import threading, subprocess, os
from numpy import matrix
import ltlmopMsg_pb2
import CSharpAckMsg_pb2


class initHandler:
    def __init__(self,proj,robotType,IPAddress = '10.0.0.86',commPort=7400,buffer=1024):
        """
        Open sockets for communication.
        robotType (int): robot type to be used PIONEER = 1, SEGWAY = 2 (default=1)
        IPAddress (string): ip of the robot (default=10.0.0.86)
        commPort (int): port on the robot for communcation between CSharp and LTLMoP (default=7400)
        buffer (int): size of the port buffer (default=1024)
        """
        try:
            # Create proxies to access modules
            self.robocomm = _CSharpCommunicator(proj,robotType,IPAddress,commPort,buffer)
            self.robocomm.start()
            
            time.sleep(1)   # Give communicator time to start and receive first data
        except RuntimeError:
            self.robocomm.stop()
            print "(INIT) ERROR: Cannot connect to the robot."
            exit(-1)
            
    def getSharedData(self):
        # Return dictionary of module proxies for other handlers to use
        return {'robocomm':self.robocomm}
 
class _CSharpCommunicator:

    # Constructor
    def __init__(self,proj,robotType,IPAddress,commPort,buffer):
        """
        Open sockets for communication.
        """
        
        # Communication parameters
        self.IPAddress = IPAddress
        self.commPort = commPort
        self.buffer = buffer
    
        self.proj = proj
        # Communication parameters
        self.addFrom = (self.IPAddress,self.commPort)
        self.TCPSock = socket(AF_INET,SOCK_STREAM)
        self.RobotType = robotType
        self.responseStr = ""

        # Sensor Status
        self.ARTAG = []
        self.LIDAR = []
        
        

        
    def start(self):
        """
        Open sockets for communication.
         """
        # test connection to the beagle board/Create, similar to CreateBeagleInit.m
        print 'Connecting to CSharp...'
        self.TCPSock.connect(self.addFrom)
        print 'Done.'
        ltlmop_msg = ltlmopMsg_pb2.PythonRequestMsg()
        ltlmop_msg.id=1
        ltlmop_msg.robot = self.RobotType
        self.responseStr = self.sendMessage(ltlmop_msg)
        time.sleep(1)
        print "CSharp Ack: ",self.responseStr.id
        self.responseStr = ""
        



    def stop(self):
        """
        Close sockets and prevent further communication.
        """
        self.TCPSock.close()

    def sendDirection(self,direction):
        """
        Serialize and send the direction vector command to the robot.
        Also used to send velocity commands to the robot.

        direction (tuple containing 2 doubles):(x,y) or (v,w) depending on application.
        """
        # get pose handler
        self.pose_handler = self.proj.h_instance['pose']
        pose = self.pose_handler.getPose()
        ltlmop_msg = ltlmopMsg_pb2.PythonRequestMsg()
        ltlmop_msg.id=2
        ltlmop_msg.vel.v = direction[0]
        ltlmop_msg.vel.omega = direction[1]
        ltlmop_msg.pose.x = pose[0]
        ltlmop_msg.pose.y = pose[1]
        ltlmop_msg.pose.yaw = pose[2]
        
        response = self.sendMessage(ltlmop_msg)
        self.updateSensorStatus(response)
        
        
    def sendMessage(self,message):
        """
        Serializes the message and send it to CSharp Interface via TCP
        and store the response message into class object self.response

        message (ltlmopMsg_pb2.PythonRequestMsg object): protobuff object to be sent
        """
        temp_serialized = message.SerializeToString()
        sent_str = pack('!I',len(temp_serialized))+temp_serialized
        self.TCPSock.send(sent_str)
        data_length = self.TCPSock.recv(self.buffer)
        response = self.TCPSock.recv(self.buffer)
        
        return self.parseResponse(response)
    
    def parseResponse(self,encryptedMsg):
        """
        Helper method that parses the serialized string from CSharp and returns
        a Protobuff message string

        encryptedMsg: serialized message to be parsed
        """
        csharp_response = CSharpAckMsg_pb2.CSharpAckMsg()
        
        if len(encryptedMsg)>0:
            return csharp_response.FromString(encryptedMsg)
        else:
            return csharp_response
    def updateSensorStatus(self,msg):
        
        if (msg.s1 is not None and len(msg.s1.data)>0):
            # we have lidar update
            self.LIDAR = msg.s1.data
        else:
            self.LIDAR = []
            

        if (msg.s2 is not None and len(msg.s2.data)>0):
            # we have ARTAG
            self.ARTAG = msg.s2.data
        else:
            self.ARTAG = []
        #print 'Updating sensors',(self.ARTAG,self.LIDAR)

    def getARTAG(self):
        result = self.ARTAG;
        return self.ARTAG

    def getLIDAR(self):
        result = self.LIDAR
        return self.LIDAR
        
    
        

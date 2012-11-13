#!/usr/bin/env python
"""
=====================================
iRobotCreateSensor.py - Sensor Handler for iRobotCreate
=====================================
"""

import threading, subprocess, os, time, socket
from ltlmopMsg_pb2 import *
from CSharpAckMsg_pb2 import *
from struct import pack,unpack
from numpy import *
from scipy.linalg import norm

class sensorHandler:
    def __init__(self, proj, shared_data):

        self.CSharpCommunicator = proj.shared_data['robocomm']
        self.pose_handler = proj.h_instance['pose']
        self.proj = proj

        self.rfi = proj.loadRegionFile()

    ###################################
    ### Available sensor functions: ###
    ###################################
    def artag_detection(self,ARTagID,initial=False):
        """
        returns the ARTag ID currently observed
        ARTagID (int): the specific ARTag number we are looking for (default=0)
        """
        if initial:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=3
            ltlmop_msg.sensor=PythonRequestMsg.ARTAG
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            #print 'got sensor init resp ',response
            return True
        else:
            ARTAG = self.CSharpCommunicator.getARTAG()
            status = False
            if len(ARTAG)>0:
                for x in range(0,len(ARTAG)):
                    if ARTAG[x] == ARTagID:
                        status = True
            if status:
                print 'I see ARTAG:',ARTagID
            return status
    def lidar_detection(self,initial=False):
        """
        returns the LIDAR points currently observed
        """
        if initial:
            ltlmop_msg = PythonRequestMsg()
            
            ltlmop_msg.id=3
            
            ltlmop_msg.sensor=PythonRequestMsg.LIDAR
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            print 'got sensor init resp ',response
            return True
        else:

            return False

    def open_doorway(self,initial=False):
        """
        returns whether a new LIDAR has discovered a new doorway that was
        not included in the original map
        """
        if initial:
            # init LIDAR at the CSharp end
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=3
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            # for now assume each region is a face
            self.exposedFaces = self.proj.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
            print 'exposedFaces:',len(self.exposedFaces)
            for face in self.exposedFaces:
                #new_p = ltlmop_msg.Point()
                new_f = ltlmop_msg.Face()
                face_p1 = self.proj.coordmap_map2lab(face[0])
                face_p2 = self.proj.coordmap_map2lab(face[1])
                new_f.p1.x = face_p1[0]
                new_f.p1.y = face_p1[1]
                #new_f.p1 = new_p # add first point
                new_f.p2.x = face_p2[0]
                new_f.p2.y = face_p2[1]
                #new_f.p2 = new_p # add second point                
                ltlmop_msg.exfaces.faces.extend([new_f]) # add face to list
            for r in self.rfi.regions:
                new_r = ltlmop_msg.Region()
                points = map(self.proj.coordmap_map2lab,r.getPoints())
                holes = [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]
                for p in points:
                    new_p = ltlmop_msg.Point()
                    new_p.x = p[0]
                    new_p.y = p[1]
                    new_r.points.extend([new_p])
                for h in holes:
                    new_h = ltlmop_msg.Point()
                    new_h.x = h[0]
                    new_h.y = h[1]
                    new_r.holes.extend([new_h])
                #print 'holes size',len(new_r.holes)
                ltlmop_msg.map.r.extend([new_r])
                #print 'region added:',len(ltlmop_msg.map.r)
                #print 'regions',ltlmop_msg.map.r
                #print 'region count',len(self.rfi.regions)
                #ltlmop_msg.map.x = map(self.proj.coordmap_map2lab,r.getPoints())
                #ltlmop_msg.map.y = [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]
            
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            
            print 'got sensor init resp '#,ltlmop_msg.map#,response
            return True
        else:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=3
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            if response.s1.data[2]<0.4:
                print "I see a door!"
                #print 'response: ',(response.s1.data[0],response.s1.data[1],response.s1.data[2],response.s1.data[3])
            return response.s1.data[2]<0.4

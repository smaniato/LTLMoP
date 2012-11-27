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
from regions import Point, Color
import copy
from specCompiler import SpecCompiler
import json

class sensorHandler:
    def __init__(self, proj, shared_data):

        self.CSharpCommunicator = proj.shared_data['robocomm']
        self.pose_handler = proj.h_instance['pose']
        self.proj = proj

        self.rfi = proj.loadRegionFile()

        # sensor variables
        self.exploreDone = False

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

    def busyExploring(self,initial=False):
        """
        returns true if we just requested an explore actuation to C#
        responsible for checking to see if C# is done exploring and
        setting the appropriate flag in LTLMoP
        """
        if initial:
            return True
        else:
            #ltlmop_msg = PythonRequestMsg()
            #ltlmop_msg.id=3
            #ltlmop_msg.sensor = PythonRequestMsg.NOSENSOR
            #ltlmop_msg.actuator = ltlmop_msg.Actuator() # needs to be changed!
            #ltlmop_msg.actuator.actuatorType = PythonRequestMsg.EXPLORE
            #ltlmop_msg.actuator.status = PythonRequestMsg.REQ_UPDATE
            #print 'dudu'
            #response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            #print 'actuator status',response.actuator.status
            return self.CSharpCommunicator.getExploreBusy()
        
        
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
                new_r.name = r.name
                #print 'holes size',len(new_r.holes)
                ltlmop_msg.map.r.extend([new_r])
            
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            # start the map update thread
            self.mapThread = _MapUpdateThread(self.proj)
            self.mapThread.daemon = True
            self.mapThread.start()
            
            print 'got sensor init resp '#,ltlmop_msg.map#,response
            return True
        else:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=3
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            if self.mapThread.mapReady:
                # for now add the new region to ltlmsg and update C#
                self.exposedFaces = self.mapThread.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
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
                for r in self.mapThread.rfi.regions:
                    new_r = ltlmop_msg.Region()
                    new_r.name = r.name
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
                    
                    print 'name: ',r.name
                    ltlmop_msg.map.r.extend([new_r])
                response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
                self.mapThread.mapReady = False
                return True
            else:
                response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
                if len(response.map.r)==0:
                    print "region received is empty!"
                    
                else:
                    print "NEW MAP RECEIVED"
                    self.mapThread.updateMap(response.map.r)
                return False

    def regionAdded(self, initial):
        """ Return true if a region was added in the last map update """

        if initial:
            return

        if self.mapThread.regionAddedFlag.isSet():
            self.addedRegions = copy.deepcopy(self.mapThread.addedRegions)
            self.mapThread.regionAddedFlag.clear()
            return True
        else:
            return False

    def regionRemoved(self, initial):
        """ Return true if a region was removed in the last map update """

        if initial:
            return

        if self.mapThread.regionRemovedFlag.isSet():
            self.removedRegions = copy.deepcopy(self.mapThread.removedRegions)
            self.mapThread.regionRemovedFlag.clear()
            return True
        else:
            return False

class _MapUpdateThread(threading.Thread):
    def __init__(self, proj, *args, **kwds):
        self.map_basename = proj.getFilenamePrefix()
        self.project_root = proj.project_root
        self.regionList = set([r.name for r in proj.rfiold.regions])
        self.regionAddedFlag = threading.Event()
        self.proj = proj
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.gotNewMap = False
        self.new_map = []
        self.rfi = []
        self.mapReady = False;

        super(_MapUpdateThread, self).__init__(*args, **kwds)
    def run(self):
        import regions
        print "running..."
        map_number = 1
        data = []
        while True:
            if self.gotNewMap:
                print "GOTnEWmAP!@"
                self.rfi = regions.RegionFileInterface(transitions=[])

                self.rfi.regions = []
                # first convert the map from CSharpAckMsg to a list of regions
                for reg in self.new_map:
                    # convert each region into json
                    #data{'name':reg.name,
                    #     'color':(0,255,0),
                    #     }
                    #data['points'] = []
                    #for p in reg.points: # add all the points of the region in lab coordinates
                    #data['points'].append((p.x,p.y))
                    #r = regions.Region()
                    #r.setData(data)
                    # transform the points to map coordinates
                    # convert each region
                    r = regions.Region()
                    r.type = 1
                    r.pointArray = []
                    r.name = reg.name
                    count = 0
                    #print 'name:',reg.name,len(reg.points),len(r.pointArray)
                    for p in reg.points: # add all the points of the region in lab coordinates
                        #print (p.x,p.y)
                        #print 'transformed',self.coordmap_lab2map(regions.Point(p.x,p.y))
                        transformed_p = self.coordmap_lab2map(regions.Point(p.x,p.y))
                        #print 'transformed:',(transformed_p),len(r.pointArray),count
                        r_pos = r.position
                        r.addPoint(regions.Point(round(transformed_p[0]-r_pos.x),round(transformed_p[1]-r_pos.y)),count)
                        #r.addPoint(regions.Point(p.x,p.y),count)
                        count = count + 1
                        #data['points'].append((p.x,p.y))
                    #r.setData(data)
                    if (len(reg.points)>0):
                        # transform the points to map coordinates
                        #r.pointArray = [self.coordmap_lab2map(*p) for p in r.pointArray]
                        
                        #r.pointArray = [Point(*p) for p in r.pointArray]
                        #r.recalcBoundingBox()
                        #print 'points added:',len(r.pointArray)
                        self.rfi.regions.append(r)
                    #else:
                    #    #print "i got an empty region for some reason!",reg
                # recompute the boundary
                self.rfi.doMakeBoundary()

                # When we receive one, write it to a new regions file 
                reg_filename = "%s.update%d.regions" % (self.map_basename, map_number)

                self.rfi.writeFile(reg_filename)

                print "Wrote region file %s." % reg_filename

                # Tell simGUI to update
                print "REGIONS:" + reg_filename

                newRegionList = set([r.name for r in self.rfi.regions])

                # Check for any substantive changes
                self.addedRegions = newRegionList - self.regionList
                self.removedRegions = self.regionList - newRegionList
                map_number += 1
                self.gotNewMap = False
                print "added: " + str(self.addedRegions)
                self.mapReady = True
                #for r in rfi.regions:
                #    for r_old in self.proj.rfi.regions:
                #        if not(r_old.name == r.name):
                #            #print "NEW REGION:%d,%d,%d,%d"%(r.pointArray[0].x,r.pointArray[0].y,r.pointArray[1].x,r.pointArray[1].y)
                #            print "new_r:%d,%d,%d,%d"%(r.pointArray[0].x,r.pointArray[0].y,r.pointArray[1].x,r.pointArray[1].y)
                
                #self.proj.rfi.regions = copy.deepcopy(rfi.regions)
                
                #compiler = SpecCompiler(self.proj.getFilenamePrefix() + ".spec")
                #print "re-decompose region"
                #compiler._decompose()
                #self.proj = compiler.proj
                #self.decomposedRFI = compiler.parser.proj.rfi
                #compiler._writeSMVFile()
                #print "done compiling :D"


    def updateMap(self, new_map):
        # store the updated map received from CSharpAckMsg
        print "updated map!"
        self.new_map = new_map
        self.gotNewMap = True
        
        
        

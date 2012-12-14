#!/usr/bin/env python
import socket, sys, os, json
from struct import *
import threading, time
from regions import Point, Color
import copy

"""
===========================================
skeletonSensor.py - Skeleton Sensor Handler
===========================================
"""

class _MapUpdateThread(threading.Thread):
    def __init__(self, proj, port, *args, **kwds):
        sys.path.append(os.path.join(proj.ltlmop_root, "lib"))

        self.port = port
        self.map_basename = proj.getFilenamePrefix() 
        self.project_root = proj.project_root
        self.regionList = set([r.name for r in proj.rfiold.regions])
        self.regionAddedFlag = threading.Event()
        self.addedRegions = []
        self.regionRemovedFlag = threading.Event()
        self.removedRegions = []
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.proj = proj

        super(_MapUpdateThread, self).__init__(*args, **kwds)

    def _getCurrentRegionFromPose(self, rfi=None):
        # TODO: this already exists in execute.py, deduplicate
        if rfi is None:
            rfi = self.proj.rfi

        pose = self.proj.h_instance['pose'].getPose()

        region = None

        for i, r in enumerate(rfi.regions):
            if r.name.lower() == "boundary":
                continue

            if r.objectContainsPoint(*self.proj.coordmap_lab2map(pose)):
                region = i
                break

        if region is None:
            print "Pose of ", pose, "not inside any region!"

        return region

    def run(self):
        import regions

        # Wait for any new maps
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("", self.port))

        map_number = 1

        while True:
            s.listen(0)
            conn, addr = s.accept()
            mapdata = conn.recv(65535)
            conn.close()

            rfi = regions.RegionFileInterface(transitions=[])

            try:
                r_data = json.loads(mapdata)
            except Exception as e:
                print "Error parsing map data!", e
                continue

            rfi.regions = []
            for reg in r_data:
                r = regions.Region()
                r.setData(reg)
                r.pointArray = map(self.coordmap_lab2map, r.pointArray)
                r.pointArray = [Point(*p) for p in r.pointArray]
                r.recalcBoundingBox()
                rfi.regions.append(r)

            # When we receive one, write it to a new regions file 
            reg_filename = "%s.update%d.regions" % (self.map_basename, map_number)

            rfi.writeFile(reg_filename)

            print "Wrote region file %s." % reg_filename

            # Tell simGUI to update
            print "REGIONS:" + reg_filename

            newRegionList = set([r.name for r in rfi.regions])

            # Check for any substantive changes
            self.addedRegions = newRegionList - self.regionList
            self.removedRegions = self.regionList - newRegionList
            
            # vvv this code doesn't work unless both functions are subscribed to vvv
            # Wait for the any old flags to be received by the other thread
            #if self.regionAddedFlag.isSet() or self.regionRemovedFlag.isSet():
            #    print "WARNING: received new map before previous delta was fully processed"
            #while self.regionAddedFlag.isSet() or self.regionRemovedFlag.isSet():
            #    time.sleep(0.01)

            if self.addedRegions:
                print "added: " + str(self.addedRegions)
                self.regionAddedFlag.set()
            if self.removedRegions:
                print "removed: " + str(self.removedRegions)
                self.regionRemovedFlag.set()

            self.regionList = newRegionList
            
            # Reload the motion control handler if only the size of the region changed
            # WARNING: This doesn't really make sense for motion controllers that require convex (decomposed) regions
            #if not self.addedRegions and not self.removedRegions:
            oldReg = self._getCurrentRegionFromPose(self.proj.rfi)
            newReg = self._getCurrentRegionFromPose(rfi)

            oldName = self.proj.rfi.regions[oldReg].name
            self.proj.rfi.regions[oldReg] = copy.deepcopy(rfi.regions[newReg])
            self.proj.rfi.regions[oldReg].name = oldName

            print "Reloading motion control handler..."
            self.proj.importHandlers(['motionControl'])

            map_number += 1

class sensorHandler:
    def __init__(self, proj, shared_data, map_listen_port, explore_done_poll_port, cup_poll_port):
        """
        Sensor handler for communicating with C# program on Pioneer.

        map_listen_port (int): TCP port to receive map updates on (default=12345)
        explore_done_poll_port (int): TCP port to poll for 'explore_done' sensor value (default=11012)
        cup_poll_port (int): UDP port to poll for 'cup' sensor value (default=11016)
        """
        try:
            self.host = shared_data['PIONEER_ADDRESS']
        except KeyError:
            print "ERROR: You need to call Pioneer Init handler before using Pioneer Actuator Handler"
            return
            
        self.ports = {'explore_done': int(explore_done_poll_port),
                      'cup': int(cup_poll_port),
                      'map': int(map_listen_port)}

        self.sensor_cache = {}

        self.proj = proj
        
        self.mapThread = _MapUpdateThread(proj, self.ports['map'])
        self.mapThread.daemon = True
        self.mapThread.start()

        self.addedRegions = []
        self.removedRegions = []


                
    def _send(self, s, msg):
        totalsent = 0
        while totalsent < len(msg):
            sent = s.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def exploreDone(self, initial):
        """ Returns true, once, if the explore_room action has been finished """
        
        if initial:
            return
        
        # TODO: don't reconnect each time
        self.explore_done_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.explore_done_socket.settimeout(1)

        # request data
        #self._send(self.explore_done_socket, 'explore_done\n')
        #print "Connected to server"
        
        try:
            self.explore_done_socket.connect((self.host, self.ports['explore_done']))
            # receive data
            data = self.explore_done_socket.recv(1)
            #print "got data: " + repr(data)
        except socket.timeout:
            print "WARNING: timeout receiving from sensor 'explore_done'"
            val = False
        else:
            val = (data == "T")

        self.explore_done_socket.close()

        if val:
            print "explore done is true"
        
        return val

    def cupSensor(self, initial):
        if initial:
            self.cup_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.cup_socket.settimeout(1, )
            self.sensor_cache['cup'] = False
            return

        self.cup_socket.sendto("?", (self.host, self.ports['cup']))
        try:
            data = self.cup_socket.recv(1)
        except socket.timeout:
            print "WARNING: timeout receiving from sensor 'cup'"
        else:
            self.sensor_cache['cup'] = (data == "T")

        return self.sensor_cache['cup']

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

       



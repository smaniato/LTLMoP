#!/usr/bin/env python

import socket
import os
import json, struct

import sys, time

class initHandler:
    def __init__(self, proj, host, map_port):
        """
        Connects to C# application on Pioneer
        
        host (string): IP address of pioneer
        map_port (int): Port to use for sending map
        """
        
        print 'preparing to send map'

        self.addr = (host, map_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Get non-decomposed region data
        rfi = proj.loadRegionFile()
        r_list = []
        for r in rfi.regions:
            r_data = {'name': r.name,
                      'color': (r.color.Red(), r.color.Green(), r.color.Blue()),
                      'points': map(proj.coordmap_map2lab, r.getPoints()),
                      'holeList': [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]}
            r_list.append(r_data)
        
        data_string = json.dumps(r_list)
        
        print "map is %d bytes long" % len(data_string)
        
        #print data_string.replace('"', '\\"')

        self._sendWithAck(struct.pack("<I", len(data_string)))
        self._sendWithAck(data_string)
        
        print 'map sent'
        
    def _sendWithAck(self, msg):
        # TODO: just use TCP for this
        self.sock.settimeout(0.5)
        
        totalsent = 0
        while totalsent < len(msg):
            success = False
            while not success:
                sent = self.sock.sendto(msg[totalsent:], self.addr)
                
                # Wait up to a half second for ack
                try:
                    self.sock.recv(1024)
                    success = True
                except socket.timeout:
                    print "ACK not received.  Retrying..."
      
            totalsent = totalsent + sent             

    def getSharedData(self):
        return {"PIONEER_ADDRESS": self.addr[0]}

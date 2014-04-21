#!/usr/bin/env python
"""
=================================================
SegwayInit.py - Segway Initialization Handler
=================================================


"""

class SegwayInitHandler:
    def __init__(self, proj, ip="10.0.0.91", port=50007):
        """
        ip (string): The ip address of Segway (default="10.0.0.91")
        port (int): The port of Segway server (default=50007)
        """
        self.ip = ip
        self.port = port
            
    def getSharedData(self):
        return {"SegwayIP":self.ip, "SegwayPort":self.port}

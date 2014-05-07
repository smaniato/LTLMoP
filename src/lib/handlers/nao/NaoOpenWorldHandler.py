#!/usr/bin/env python
"""
=====================================
NaoOpenWorldHandler.py - 
=====================================

Use Nao's low-level sensors to generate new propositions
"""

import threading, subprocess, os, time, socket
import numpy, math
import sys

import lib.handlers.handlerTemplates as handlerTemplates
import logging

class NaoOpenWorldHandler(handlerTemplates.OpenWorldHandler):
    def __init__(self, executor, shared_data):
                
        self.incrementTrack = {}
        self.executor = executor
                
    #Return:
    #   The name of the new proposition
    #   A tuple consisting of: (param_to_change, new_param) for sensor affiliated with group       
    def dumb(self,root_name,initial=False):
        """
        Return "root_name_num"
        root_name (string): Root name of proposition to be returned

        """
        if initial:
            return True
        else:
            prop_name = "kevin"
            sensor_info = ("button_name", prop_name)
            return (prop_name, sensor_info)
    

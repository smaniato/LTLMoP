#!/usr/bin/env python
"""
=====================================
DummyOpenWorldHandler.py - Dummy Sensor Handler
=====================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time, socket
import numpy, math
import sys

import lib.handlers.handlerTemplates as handlerTemplates
import logging

class DummyOpenWorldHandler(handlerTemplates.OpenWorldHandler):
    def __init__(self, executor, shared_data):
                
        self.incrementTrack = {}
        self.executor = executor
                
    #Return:
    #   The name of the new proposition
    #   A tuple consisting of: (param_to_change, new_param) for sensor affiliated with group       
    def increment(self,root_name,detect_sensor=None,initial=False):
        """
        Return "root_name_num"
        root_name (string): Root name of proposition to be returned
        detect_sensor (string): Name of sensor proposition that is being used as the detector
        """
        if initial:
            return True
        else:
            if root_name in self.incrementTrack:
                self.incrementTrack[root_name] = self.incrementTrack[root_name] + 1
            else:
                self.incrementTrack[root_name] = 1
            prop_name = root_name + str(self.incrementTrack[root_name])
            sensor_info = (detect_sensor,"button_name", prop_name)
            return (prop_name, sensor_info)
    
    
    def queryUser(self, question,detect_sensor=None, initial=False):
        """
        Query User for response
        question (string): Question to ask user
        detect_sensor (string): Name of sensor proposition that is being used as the detector
        """
        if initial:
            return True
        else:
            self.executor.user_query_response = None
            default_response="blank"
            accept_empty = False
            while self.executor.user_query_response is None or \
             self.executor.user_query_response.strip() == "" and not accept_empty:
                self.executor.received_user_query_response.clear()
                self.executor.postEvent("QUERY_USER", [question, default_response])
                self.executor.received_user_query_response.wait()
            prop_name = self.executor.user_query_response
            sensor_info = (detect_sensor,"button_name", prop_name)
            return (prop_name, sensor_info)
        #        accept_empty (bool): Whether or not to accept empty response
        


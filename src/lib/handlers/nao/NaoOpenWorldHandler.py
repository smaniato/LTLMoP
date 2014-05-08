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
        self.shared_data = shared_data
                
    def queryUser(self, question,detector=None,detectorParam=None,initial=False):
        """
        Query User for response
        question (string): Question to ask user
        detector (string): Name of sensor proposition that is being used as the detector
        detectorParam (string): Name of parameter from detector to be changed for new propositions
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

            sensor_info = (detector,detectorParam, str(self.shared_data['detectVal']))
            self.shared_data['detectVal'] = None
            #sensorInfo[0] = name of sensor propotion being modeled
            #sensorInfo[1] = name of parameter to be changed
            #sensorInfo[2] = value of parameter
            return (prop_name, sensor_info)
        #        accept_empty (bool): Whether or not to accept empty response
    

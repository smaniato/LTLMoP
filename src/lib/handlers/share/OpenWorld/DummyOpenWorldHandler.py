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
                
    def increment(self,root_name,initial=False):
        """
        Return "root_name_num"
        root_name (string): Root name of proposition to be returned

        """
        if initial:
            return True
        else:
            if root_name in self.incrementTrack:
                self.incrementTrack[root_name] = self.incrementTrack[root_name] + 1
            else:
                self.incrementTrack[root_name] = 1
            return root_name + str(self.incrementTrack[root_name])
    
    
    def queryUser(self, question, initial=False):
        """
        Query User for response
        question (string): Question to ask user

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
        
            return self.executor.user_query_response
        #        accept_empty (bool): Whether or not to accept empty response
        


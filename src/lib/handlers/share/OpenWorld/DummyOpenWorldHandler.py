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
        logging.info("Intializing DummyOpenWorld")
    
    def increment(self,root_name,initial=False):
        """
        Return "root_name_num"
        root_name (string): Root name of proposition

        """
        if initial:
            logging.info("called with intial TRUE")

        return root_name
        
    
        # self.user_query_response = ""
        # self.recieved_user_query_response = threading.Event()
         
    # def _queryUser(self, question, default_response="", accept_empty=False):
    #     self.user_query_response = None
    #     
    #     while self.user_query_response is None or \
    #      self.user_query_ersponse.strip() == "" and not accept_empty:
    #         self.recieved_user_query_response.clear()
    #         self.postEvent("QUERY_USER", [question, default_response])
    #         self.recieved_user_query_response.wait()
    #     
    #     return self.user_query_response
    #     
    # def propFromUser(self):
    #     response = self._queryUser("What should I add?")
    #     
    #     return response



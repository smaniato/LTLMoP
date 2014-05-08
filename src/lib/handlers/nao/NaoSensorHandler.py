#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""

import lib.handlers.handlerTemplates as handlerTemplates
import logging

class NaoSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']
        self.shared_data = shared_data
        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None
        self.memProxy = None
        self.sttProxy = None
        self.ldmProxy = None

        self.landMarkInitialized = False

    ###################################
    ### Available sensor functions: ###
    ###################################
    def _initLandMark(self):
        if not self.landMarkInitialized:
        # initialize landmark detection
            if self.ldmProxy == None:
                self.ldmProxy = self.naoInitHandler.createProxy('ALLandMarkDetection')
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

        ### Initialize land Mark tracking
            subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]

        # Close any previous subscriptions that might have been hanging open
            if "ltlmop_sensorhandler" in subs:
                self.ldmProxy.unsubscribe("ltlmop_sensorhandler")
            self.ldmProxy.subscribe("ltlmop_sensorhandler", 100, 0.0)
            self.landMarkInitialized=True

    def _getLandMarkNum(self):
        foundMarks=[]
        val = self.memProxy.getData("LandmarkDetected",0)
        if(val and isinstance(val, list) and len(val)>=2):
            markInfoArray = val[1]
            try:
                #msg = "Number of Markers found: " + str(len(markInfoArray))
                #msg = "length of foundMarks: "
                for markInfo in markInfoArray:
                    markExtraInfo = markInfo[1] #Number of each tag found
                    foundMarks.append(markExtraInfo[0])
                    #msg += "-- " + str(len(foundMarks))
                #logging.info(msg)

                #Filter out crappy readings from the Nao. 
                filterList = [68,170,130]
                finalList = []
                for item in foundMarks:
                    if (item in filterList):
                        finalList.append(item)
                
                return finalList

            except Exception, e:
                print "Naomarks detected, but it seems getData is invalid. ALValue ="
                print val
                print "Error msg %s" % (str(e))

    def seeLandMark(self,landMark_id,initial=False):
        """
        Use Nao's landmark recognition system to detect radial bar code landmark.
        For info about avaible bar code, refer to http://www.aldebaran-robotics.com/documentation/naoqi/vision/allandmarkdetection.html#allandmarkdetection

        landMark_id (int): The id number of bar code to detect
        """
        if initial:
            self._initLandMark()
            return True
        else:
            allFound = self._getLandMarkNum()
            if (isinstance(allFound, list) and len(allFound)>0):
                if (landMark_id in allFound):
                    return True
            return False

    def _resetKnownVals(self):
        for k,v in self.knownVals.iteritems():
            self.knownVals[k] = False

    def detectLandMark(self,detectVal,detector=False, initial=False):
        """
        Use Nao's landmark recognition system to detect radial bar code landmark.
        For info about avaible bar code, refer to http://www.aldebaran-robotics.com/documentation/naoqi/vision/allandmarkdetection.html#allandmarkdetection
        detector (bool): Whether or not the function should act as a detector
        detectVal (int): The id number of bar code to detect
        """
        if initial:
            self._initLandMark()
            self.knownVals = {}
            self.shared_data['detectVal'] = None
            logging.info("detectLandMark Initialized")
            return True
        else:
            if detector==True:
                allFound = self._getLandMarkNum()
                if not (isinstance(allFound, list) and len(allFound)>0):
                    allFound = []

                self._resetKnownVals()
                for foundElem in allFound:
                    if foundElem in self.knownVals:
                        self.knownVals[foundElem] = True
                    else:
                        self.shared_data['detectVal'] = foundElem
                        return True
                return False
            elif detectVal in self.knownVals:
                return self.knownVals[detectVal]
            else:
                self.knownVals[detectVal] = True
                return True
            



    def hearWord(self, word, threshold, initial=False):
        """
        Use Nao's speech recognition system to detect a spoken word.

        word (string): The word to detect
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)
        """

        if initial:
            ### Initialize speech-to-text

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            if self.sttProxy is None:
                self.sttProxy = self.naoInitHandler.createProxy('ALSpeechRecognition')

            # Close any previous subscriptions that might have been hanging open
            subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
            if "ltlmop_sensorhandler" in subs:
                self.sttProxy.unsubscribe("ltlmop_sensorhandler")

            self.sttVocabulary += [word]
            self.sttProxy.setWordListAsVocabulary(self.sttVocabulary)

            self.sttProxy.setLanguage("English")
            self.sttProxy.setAudioExpression(False)
            self.sttProxy.setVisualExpression(True)
            self.sttProxy.subscribe("ltlmop_sensorhandler")

            # Reset the speech recognition register manually
            self.memProxy.insertData("WordRecognized", [])

            return True
        else:
            # Check speech recognition state

            wds = self.memProxy.getData("WordRecognized",0)

            # HACK: reset the speech recognition register manually once per vocab-cycle
            self.sttVocabCounter += 1
            if self.sttVocabCounter == len(self.sttVocabulary):
                self.memProxy.insertData("WordRecognized", [])
                self.sttVocabCounter = 0

            for wd, prob in zip(wds[0::2], wds[1::2]):
                if wd == word and prob > threshold:
                    print "Recognized word '%s' with p = %f" % (wd, prob)
                    return True

            return False


    def seePerson(self, initial=False):
        """
        Use Nao's face recognition to detect a person's face in the field of view.
        """

        if initial:
            ### Initialize face tracking

            if self.faceProxy is None:
                self.faceProxy = self.naoInitHandler.createProxy('ALFaceDetection')

                subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
                # Close any previous subscriptions that might have been hanging open
                if "ltlmop_sensorhandler" in subs:
                    self.faceProxy.unsubscribe("ltlmop_sensorhandler")
                self.faceProxy.subscribe("ltlmop_sensorhandler")

                return True
        else:
            # Check face detection state
            face_data = self.memProxy.getData("FaceDetected",0)
            return (face_data != [])

    def headTapped(self, initial=False):
        """
        Check whether the button on top of Nao's head is pressed.
        """

        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            return True
        else:
            return bool(self.memProxy.getData('FrontTactilTouched',0))


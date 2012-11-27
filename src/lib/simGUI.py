#!/usr/bin/env python
# -*- coding: utf-8 -*-
# generated by wxGlade 0.6.3 on Sun Jan 31 10:48:54 2010

"""
    ==================================
    simGUI.py - Experiment Monitor GUI
    ==================================

    A basic user interface for watching the state of the robot during simulation/experiment,
    and pausing/resuming execution.
"""

import math, time, sys, os, re
import wxversion
import wx, wx.richtext, wx.grid
import threading
import project, mapRenderer, regions
from socket import *

# begin wxGlade: extracode
# end wxGlade

class SimGUI_Frame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: SimGUI_Frame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.window_1 = wx.SplitterWindow(self, -1, style=wx.SP_3D|wx.SP_BORDER|wx.SP_LIVE_UPDATE)
        self.window_1_pane_2 = wx.Panel(self.window_1, -1)
        self.sizer_2_copy_staticbox = wx.StaticBox(self.window_1_pane_2, -1, "Status Log")
        self.window_1_pane_1 = wx.Panel(self.window_1, -1)
        self.text_ctrl_sim_log = wx.richtext.RichTextCtrl(self.window_1_pane_2, -1, "", style=wx.TE_MULTILINE)
        self.button_sim_startPause = wx.Button(self.window_1_pane_2, -1, "Start")
        self.button_sim_log_clear = wx.Button(self.window_1_pane_2, -1, "Clear Log")
        self.button_sim_log_export = wx.Button(self.window_1_pane_2, -1, "Export Log...")
        self.label_1 = wx.StaticText(self.window_1_pane_2, -1, "Show log messages for:")
        self.checkbox_statusLog_targetRegion = wx.CheckBox(self.window_1_pane_2, -1, "Target region announcements")
        self.checkbox_statusLog_propChange = wx.CheckBox(self.window_1_pane_2, -1, "System proposition changes")
        self.checkbox_statusLog_border = wx.CheckBox(self.window_1_pane_2, -1, "Region border crossings")
        self.checkbox_statusLog_other = wx.CheckBox(self.window_1_pane_2, -1, "Other debugging messages")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.onSimStartPause, self.button_sim_startPause)
        self.Bind(wx.EVT_BUTTON, self.onSimClear, self.button_sim_log_clear)
        self.Bind(wx.EVT_BUTTON, self.onSimExport, self.button_sim_log_export)
        self.Bind(wx.EVT_SPLITTER_SASH_POS_CHANGED, self.onResize, self.window_1)
        # end wxGlade
        self.window_1_pane_1.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.mapBitmap = None

        self.window_1_pane_1.Bind(wx.EVT_PAINT, self.onPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.onEraseBG)

        self.proj = project.Project()
        self.proj.setSilent(True)

        # Make status bar at bottom.

        self.sb = wx.StatusBar(self)
        self.SetStatusBar(self.sb)
        self.sb.SetFieldsCount(1)
        self.sb.SetStatusText("PAUSED")

        self.button_sim_log_export.Enable(False)

        #??# Set up socket for communication to executor
        print "(GUI) Starting socket for communication to controller"
        self.host = 'localhost'
        self.portTo = 9562
        self.buf = 1024
        self.addrTo = (self.host,self.portTo)
        self.UDPSockTo = socket(AF_INET,SOCK_DGRAM)
        
        #??# Set up socket for communication from executor
        print "(GUI) Starting socket for communication from controller"
        self.portFrom = 9563
        self.addrFrom = (self.host,self.portFrom)
        self.UDPSockFrom = socket(AF_INET,SOCK_DGRAM)
        self.UDPSockFrom.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.UDPSockFrom.bind(self.addrFrom)

        # Create new thread to communicate with subwindow
        print >>sys.__stdout__, "(GUI) Starting controller listen thread..."
        self.controllerListenThread = threading.Thread(target = self.controllerListen)
        self.controllerListenThread.start()
    
        self.robotPos = None
        self.robotVel = (0,0)

        self.markerPos = None

        # Let everyone know we're ready
        self.UDPSockTo.sendto("Hello!",self.addrTo)

    def loadRegionFile(self, filename):
        self.proj.rfi = regions.RegionFileInterface()
        self.proj.rfi.readFile(filename)

        self.Bind(wx.EVT_SIZE, self.onResize, self)
        self.onResize()

    def loadSpecFile(self, filename):
        self.proj.loadProject(filename)

        self.Bind(wx.EVT_SIZE, self.onResize, self)
        self.onResize()

    def controllerListen(self):
        """
        Processes messages from the controller, and updates the GUI accordingly
        """

        ############################
        # CONTROLLER LISTEN THREAD #
        ############################

        while 1: 
            # Wait for and receive a message from the controller
            input,self.addrFrom = self.UDPSockFrom.recvfrom(self.buf)
            if input == '':  # EOF indicates that the connection has been destroyed
                print "Controller listen thread is shutting down."
                break

            input = input.strip()
            #print >>sys.__stdout__, input

            # Update stuff (should put these in rough order of frequency for optimal speed
            if input.startswith("Running at"):
                wx.CallAfter(self.sb.SetStatusText, input, 0)
            elif input.startswith("POSE:"):
                [x,y] = map(float, input.split(":")[1].split(","))
                self.robotPos = (x, y)
                wx.CallAfter(self.onPaint)
            elif input.startswith("marker:"):
                [x,y] = map(float, input.split(":")[1].split(","))
                self.markerPos = (x, y)
                wx.CallAfter(self.onPaint)
            elif input.startswith("VEL:"):
                [x,y] = map(float, input.split(":")[1].split(","))
                [x,y] = map(int, (self.mapScale*x, self.mapScale*y)) 
                self.robotVel = (x, y)
            elif input.startswith("PAUSE"):
                # FIXME: Sometimes we'll still get rate updates AFTER a pause
                wx.CallAfter(self.sb.SetStatusText, input, 0)
            elif input.startswith("Output proposition"):
                if self.checkbox_statusLog_propChange.GetValue():
                    wx.CallAfter(self.appendLog, input + "\n", color="GREEN") 
            elif input.startswith("Heading to"):
                if self.checkbox_statusLog_targetRegion.GetValue():
                    wx.CallAfter(self.appendLog, input + "\n", color="BLUE") 
            elif input.startswith("Crossed border"):
                if self.checkbox_statusLog_border.GetValue():
                    wx.CallAfter(self.appendLog, input + "\n", color="CYAN") 
            elif input.startswith("SPEC:"):
                print "LOADING SPEC"
                wx.CallAfter(self.loadSpecFile, input.split(":",1)[1])
            elif input.startswith("REGIONS:"):
                print "LOADING REGIONS"
                wx.CallAfter(self.loadRegionFile, input.split(":",1)[1])
            else:
                if self.checkbox_statusLog_other.GetValue():
                    if input != "":
                        wx.CallAfter(self.appendLog, input + "\n", color="BLACK") 

    def __set_properties(self):
        # begin wxGlade: SimGUI_Frame.__set_properties
        self.SetTitle("Simulation Status")
        self.SetSize((836, 713))
        self.checkbox_statusLog_targetRegion.SetValue(1)
        self.checkbox_statusLog_propChange.SetValue(1)
        self.checkbox_statusLog_border.SetValue(1)
        self.checkbox_statusLog_other.SetValue(1)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SimGUI_Frame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_43_copy_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_3 = wx.BoxSizer(wx.VERTICAL)
        sizer_43_copy_copy = wx.BoxSizer(wx.VERTICAL)
        sizer_2_copy = wx.StaticBoxSizer(self.sizer_2_copy_staticbox, wx.HORIZONTAL)
        sizer_5.Add((20, 30), 0, 0, 0)
        sizer_43_copy_copy.Add((20, 20), 0, 0, 0)
        sizer_2_copy.Add(self.text_ctrl_sim_log, 1, wx.ALL|wx.EXPAND, 2)
        sizer_43_copy_copy.Add(sizer_2_copy, 1, wx.EXPAND, 0)
        sizer_43_copy_copy.Add((20, 20), 0, 0, 0)
        sizer_5.Add(sizer_43_copy_copy, 6, wx.EXPAND, 0)
        sizer_5.Add((20, 30), 0, 0, 0)
        sizer_43_copy_1.Add((20, 20), 0, 0, 0)
        sizer_43_copy_1.Add(self.button_sim_startPause, 0, wx.LEFT|wx.RIGHT|wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL, 20)
        sizer_43_copy_1.Add((20, 20), 0, 0, 0)
        sizer_43_copy_1.Add(self.button_sim_log_clear, 0, wx.LEFT|wx.RIGHT|wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL, 20)
        sizer_43_copy_1.Add((20, 20), 0, 0, 0)
        sizer_43_copy_1.Add(self.button_sim_log_export, 0, wx.LEFT|wx.RIGHT|wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL, 20)
        sizer_43_copy_1.Add((20, 20), 0, 0, 0)
        sizer_3.Add((20, 40), 0, 0, 0)
        sizer_3.Add(self.label_1, 0, 0, 0)
        sizer_3.Add(self.checkbox_statusLog_targetRegion, 0, wx.TOP|wx.BOTTOM, 5)
        sizer_3.Add(self.checkbox_statusLog_propChange, 0, wx.TOP|wx.BOTTOM, 5)
        sizer_3.Add(self.checkbox_statusLog_border, 0, wx.TOP|wx.BOTTOM, 5)
        sizer_3.Add(self.checkbox_statusLog_other, 0, wx.TOP|wx.BOTTOM, 5)
        sizer_43_copy_1.Add(sizer_3, 1, wx.EXPAND, 0)
        sizer_5.Add(sizer_43_copy_1, 3, wx.EXPAND, 0)
        sizer_5.Add((20, 30), 0, 0, 0)
        self.window_1_pane_2.SetSizer(sizer_5)
        self.window_1.SplitHorizontally(self.window_1_pane_1, self.window_1_pane_2)
        sizer_2.Add(self.window_1, 1, wx.EXPAND, 0)
        sizer_2.Add((20, 20), 0, 0, 0)
        sizer_1.Add(sizer_2, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

        self.window_1.SetSashPosition(self.GetSize().y/2)
        self.window_1_pane_1.SetBackgroundColour(wx.WHITE)   

    def onResize(self, event=None): # wxGlade: SimGUI_Frame.<event_handler>
        size = self.window_1_pane_1.GetSize()
        self.mapBitmap = wx.EmptyBitmap(size.x, size.y)
        self.mapScale = mapRenderer.drawMap(self.mapBitmap, self.proj.rfi, scaleToFit=True, drawLabels=True, memory=True)

        self.Refresh()
        self.Update()

        if event is not None:
            event.Skip()

    def onEraseBG(self, event):
        # Avoid unnecessary flicker by intercepting this event
        pass

    def onPaint(self, event=None):
        if self.mapBitmap is None:
            return

        if event is None:
            dc = wx.ClientDC(self.window_1_pane_1)
        else:
            pdc = wx.AutoBufferedPaintDC(self.window_1_pane_1)
            try:
                dc = wx.GCDC(pdc)
            except:
                dc = pdc

        dc.BeginDrawing()

        # Draw background
        dc.DrawBitmap(self.mapBitmap, 0, 0)

        # Draw robot
        if self.robotPos is not None:
            [x,y] = map(lambda x: int(self.mapScale*x), self.robotPos) 
            dc.DrawCircle(x, y, 5)
        if self.markerPos is not None:
            [m,n] = map(lambda m: int(self.mapScale*m), self.markerPos) 
            dc.SetBrush(wx.Brush(wx.RED))
            dc.DrawCircle(m, n, 5)

        # Draw velocity vector of robot (for debugging)
        #dc.DrawLine(self.robotPos[0], self.robotPos[1], 
        #            self.robotPos[0] + self.robotVel[0], self.robotPos[1] + self.robotVel[1])

        dc.EndDrawing()
        
        if event is not None:
            event.Skip()

    def appendLog(self, text, color="BLACK"):
        # for printing everything on the log
            
        # annotate any pXXX region names with their human-friendly name
        # convert to set to avoid infinite explosion
        for p_reg in set(re.findall(r'\b(p\d+)\b',text)):
            for rname, subregs in self.proj.regionMapping.iteritems():
                if p_reg in subregs:
                    break
            text = re.sub(r'\b'+p_reg+r'\b', '%s (%s)' % (p_reg, rname), text)

        self.text_ctrl_sim_log.BeginTextColour(color)
        self.text_ctrl_sim_log.AppendText("["+time.strftime("%H:%M:%S", time.gmtime())+"] "+text)
        self.text_ctrl_sim_log.EndTextColour()
        self.text_ctrl_sim_log.ShowPosition(self.text_ctrl_sim_log.GetLastPosition())
        self.text_ctrl_sim_log.Refresh()

    def onSimStartPause(self, event): # wxGlade: SimGUI_Frame.<event_handler>
        btn_label = self.button_sim_startPause.GetLabel()
        if btn_label == "Start" or btn_label == "Resume":
            self.button_sim_log_export.Enable(False)
            self.UDPSockTo.sendto("START",self.addrTo) # This goes to the controller
            self.appendLog("%s!\n" % btn_label,'GREEN')
            self.button_sim_startPause.SetLabel("Pause")
        else:
            self.UDPSockTo.sendto("PAUSE",self.addrTo) # This goes to the controller
            self.appendLog('Pause...\n','RED')
            self.button_sim_log_export.Enable(True)
            self.button_sim_startPause.SetLabel("Resume")

        self.Refresh()
        event.Skip()

    def onSimExport(self, event): # wxGlade: SimGUI_Frame.<event_handler>
        """
        Ask the user for a filename to save the Log as, and then save it.
        """
        default = 'StatusLog'
    
        # Get a filename
        fileName = wx.FileSelector("Save File As", 
                                    os.path.join(os.getcwd(),'examples'),
                                    default_filename=default,
                                    default_extension="txt",
                                    wildcard="Status Log files (*.txt)|*.txt",
                                    flags = wx.SAVE | wx.OVERWRITE_PROMPT)
        if fileName == "": return # User cancelled.

        # Force a .txt extension.  How mean!!!
        if os.path.splitext(fileName)[1] != ".txt":
            fileName = fileName + ".txt"
        

        # Save data to the file
        self.saveFile(fileName)
        event.Skip()

    def saveFile(self, fileName):
        """
        Write all data out to a file.
        """

        if fileName is None:
            return

        f = open(fileName,'w')

        print >>f, "Experiment Status Log"
        print >>f
        # write the log
        print >>f, str(self.text_ctrl_sim_log.GetValue())

        f.close()

    def onSimClear(self, event): # wxGlade: SimGUI_Frame.<event_handler>
        self.text_ctrl_sim_log.Clear()
        event.Skip()


# end of class SimGUI_Frame




if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    simGUI_Frame = SimGUI_Frame(None, -1, "")
    app.SetTopWindow(simGUI_Frame)
    simGUI_Frame.Show()
    app.MainLoop()

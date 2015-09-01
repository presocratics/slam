#!/usr/bin/env python2.7
# sensors.py
# Martin Miller
# Created: 2015/08/25
"""Class for sensors"""
import numpy as np
import sys

class Sensors:
    def __init__(self):
        self.quat=np.array([0,0,0,0.])
        self.ang=np.array([0,0,0.])
        self.t=-1
        self.dt=-1
        self.line=""
        self.newq=0
        self.neww=0

    def update(self):
        """Update the sensors"""
        """Start where we left off"""
        self.newq=0
        self.neww=0
        if self.line=="": # first time reading
            line=sys.stdin.readline()
            line=line.strip()
        else:
            line=self.line
        tprev,rest=line.split(",",1)
        tprev=float(tprev)
        firstT=self.t

        while True:
            t,rest=line.split(",",1)
            t=float(t)
            if t>(tprev+0.005):
                self.line=line
                self.dt=tprev-firstT
                self.t=tprev
                return True
            if "ANG" in rest:
                self.neww=1
                f,ang=rest.split(",",1)
                self.setAng(ang.split(","))
            elif "QUAT" in rest:
                self.newq=1
                f,quat=rest.split(",",1)
                self.setQuat(quat.split(","))

            line=sys.stdin.readline()
            if line=="": # EOF
                return False
            line=line.strip()
            tprev=t

    def setAng(self,w):
        """Sets angular rate"""
        self.ang=np.array(w,dtype=float)

    def setQuat(self,q):
        """Sets quaternion"""
        self.quat=np.array(q,dtype=float)


#!/usr/bin/env python2.7
# quatKF.py
# Martin Miller
# Created: 2015/08/25
"""Implements a Kalman Filter for estimating the quaternion
Input via stdin in the format:
    time,ANG,w0,w1,w2
    time,QUAT,q0,q1,q2,q3
"""
import sensors
import numpy as np
import sys

def getF(w,dt):
    """Creates F matrix"""
    Ft=np.matrix("0 %f %f %f; \
                  %f 0 %f %f; \
                  %f %f 0 %f; \
                  %f %f %f 0" %
                  (w[2],-w[1],w[0],
                  -w[2],w[0],w[1],
                  w[1],w[0],w[2],
                  -w[0],-w[1],-w[2]))
    return np.eye(4)+0.5*Ft*dt


def main():
    s=sensors.Sensors()
    s.update()
    while s.newq!=1: # Initialize quaternion
        s.update()
    q=np.matrix(s.quat).T
    P=1e-3*np.eye(4)
    R=1e-7*np.eye(4)
    Q=1e-4*np.eye(4)
    noMeas=True
    while s.update():
        if noMeas is False:
            Q=1e-4*np.eye(4)
        """Prediction"""
        F=getF(s.ang,s.dt) # F_k
        if s.newq==1:
            noMeas=False
            S=np.matrix(P+R) # S_k=P_k|k-1+R
            K=P*S.getI() # P_k|k-1
            y=np.matrix(s.quat).T-q # z-q_k|k-1 TODO: use q+?
            qkk=q+K*y # q_k|k=q_k|k-1+K*y~
            #P=(np.eye(4)-K)*P # P_k|k=(I-K)P_k|k-1
            P=(np.eye(4)-K)*P*(np.eye(4)-K).T+K*R*K.T
        P=F*P*F.T+Q # P_k|k-1=F*P_k|k*Ft+Q
        if s.newq==1:
            q=F*qkk
        else:
            q=F*q
        Q=np.array(q).flatten()
        print "%0.5f,%0.9f,%0.9f,%0.9f,%0.9f" % (s.t,Q[0],Q[1],Q[2],Q[3])
        P=0.5*P+0.5*P.T


if __name__=='__main__':
    main()


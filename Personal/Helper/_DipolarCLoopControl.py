#!/usr/bin/env python
"""
==================================================
DipolarController
==================================================

This class uses matplotlib to aid in plotting several components of
the DipolarRRT algorithm.
"""

import numpy as np
from math import sin, cos, tanh, atan2, pi, sqrt

class DipolarController:
    
    def __init__(self, k1=.5, k2=1.5, lambdaValue=3):
        """ A class that uses the dipolar controller from the paper: Model 
        Predictive Control for the Navigation of a Nonholonomic Vehicle with 
        Field-of-View Constraints
        
        :param k1: Linear velocity
        :param k2: Angular velocity gain
        :param lambdaValue: Dipole's field shape
        Parameters are further explained in the literature
        """
        self.k1 = k1
        self.k2 = k2
        self.lam = lambdaValue
    
    def getFieldAt(self, poseCurr, poseDes):
        """ Calculate the dipolar field at poseCurr that is generated from 
        the dipole at poseDes
        
        :param poseCurr: numpy 3D array. The current pose
        :param poseDes: numpy 3D array. The desired pose
        :return: fieldXVector, fieldYVector
        """
        rX = poseCurr[0] - poseDes[0]
        rY = poseCurr[1] - poseDes[1]
        pX = cos(poseDes[2])
        pY = sin(poseDes[2])
        
        a1 = self.lam*(pX*rX + pY*rY)
        a2 = rX*rX + rY*rY
        
        return (a1*rX - a2*pX, a1*rY - a2*pY)
    
        
    def getControls(self, poseCurr, poseDes, posePrev, delT):                         # No Backup
        """ Get the dipolar controls that will take the robot from poseCurr to
        poseDes.
        
        :param poseCurr: numpy 3D array. The current pose
        :param poseDes: numpy 3D array. The desired pose
        :param posePrev: numpy 3D array. The previous pose
        :param delT: The time elapsed since posePrev
        :return: linearVelocity, angularVelocity
        """        
        rX = poseCurr[0] - poseDes[0]
        rY = poseCurr[1] - poseDes[1]    
        ang = poseCurr[2]
        u = -self.k1 * sign(rX*cos(ang) + rY*sin(ang)) * \
            tanh(rX*rX + rY*rY)
#         u = -self.k1 * sign(rX*cos(ang) + rY*sin(ang))
           
        dipoleField = self.getFieldAt(poseCurr, poseDes)
        dipoleFieldPrev = self.getFieldAt(posePrev, poseDes)
        phi = atan2(dipoleField[1], dipoleField[0])
        phiPrev = atan2(dipoleFieldPrev[1], dipoleFieldPrev[0])
           
        w = -self.k2*diffAngles(ang, phi) + diffAngles(phi, phiPrev)/delT
    
        return u, w  
    
    def integrateForwards(self, posePrev, u, w, delT):
        """ Calculate the robots new position based on the previous position,
        controls, and time elapsed.
        
        :param poseDes: numpy 3D array. The previous pose
        :param u: linear velocity
        :param w: angular velocity
        :param delT: time elapsed
        :return: poseN: the new pose of the robot
        """
        poseN = np.array(posePrev, dtype=float)
        
        delAng = w*delT
        distancePointE2 = u*delT
        
        if abs(delAng) < .0000001:
            poseN[0] += distancePointE2*cos(posePrev[2])   
            poseN[1] += distancePointE2*sin(posePrev[2])
        else:
            # Radius of circle and length of displacement vector
            rad = distancePointE2/delAng;
            vecL = sqrt( (rad*sin(delAng))**2 + (rad - rad*cos(delAng))**2) * sign(distancePointE2)
            
            poseN[0] += vecL*cos(delAng/2 + poseN[2])
            poseN[1] += vecL*sin(delAng/2 + poseN[2])
            poseN[2] = (poseN[2] + delAng)%(2*pi)
            
        return poseN

def sign(x):
    return int(x > 0) - int(x < 0)
  
def diffAngles(angle1, angle2):
    """ Returns difference between -pi and pi. Relative to angle2.
    """
    return (angle1 - angle2 + pi)%(2*pi) - pi

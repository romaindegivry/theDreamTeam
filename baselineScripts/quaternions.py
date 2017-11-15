# coding=utf-8
from math import sin,cos,sqrt,pi

#TODO: Consider replacing by better package: https://github.com/KieranWynn/pyquaternion


class Quaternion:
    """
    This class represents a quaternion with some operations

    """
    def __init__(self,s,vx,vy,vz):
        self.s = s
        self.x = vx
        self.y = vy
        self.z = vz

    def __rmul__(self,q2):
        if type(q2)==type(self):

            #   |           |            |            |            |
            a1 = self.s*q2.s -self.x*q2.x -self.y*q2.y -self.z*q2.z
            a2 = self.s*q2.x +self.y*q2.z -self.z*q2.y +self.x*q2.s
            a3 = self.s*q2.y +self.y*q2.s -self.x*q2.z +self.z*q2.x
            a4 = self.s*q2.z +self.z*q2.s +self.x*q2.y -self.y*q2.x
            return Quaternion(a1,a2,a3,a4)
        elif type(q2)==float or type(q2)==int:
            return Quaternion(self.s*q2,self.x*q2,self.y*q2,self.z*q2)
        else:
            raise TypeError

    def __mul__(self,q2):
        if type(q2)==float or type(q2)==int:
            return Quaternion(self.s*q2,self.x*q2,self.y*q2,self.z*q2)
        elif type(q2)==type(self):
            return(self.__rmul__(q2))
        else:
            pass

    def __repr__(self):
        return "quaternion : [{},({},{},{})]".format(self.s,self.x,self.y,self.z)

    def conjugate(self):
        return Quaternion(self.s,-self.x,-self.y,-self.z)

    def  norm(self):
        return sqrt(self.s*self.s+self.x*self.x+self.y*self.y+self.z*self.z)
    #TODO: Include division as __rdiv__ and __div__

#let us define a vector class compatible with quaternions

class Vector (Quaternion):
    def __init__(self,x,y,z):
        self.s = 0
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "vector : ({},{},{})".format(self.x,self.y,self.z)



#generates the quaternion for a rotation around the axis given by vecor by an
#angle alpha
def rotationQuaternion(vector,alpha):
    s = sin(alpha/2.)
    v = vector*(1/vector.norm())
    return Quaternion(cos(alpha/2.),
                      s*vector.x,
                      s*vector.y,
                      s*vector.z)

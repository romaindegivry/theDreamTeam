# coding=utf-8

import sys
sys.path.append("..") # Adds higher directory to python modules path.
#see https://stackoverflow.com/questions/1054271

from quaternions import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

z = Vector(0,0,1)
x = Vector(1,0,0)
y = Vector(0,1,0)

psi = pi/12.
theta = 0.
phi = pi/12.

qPsi = rotationQuaternion(z,psi)
qTheta = rotationQuaternion(y,theta)
qPhi = rotationQuaternion(x,phi)
q = qPsi*qTheta*qPhi

xstar = q*x*q.conjugate()
ystar = q*y*q.conjugate()
zstar = q*z*q.conjugate()

print('x = {}'.format(xstar))
print('y = {}'.format(ystar))
print('z = {}'.format(zstar))

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

def plotVector(axs,vectors,color):
    U = np.array([v.x for v in vectors])
    V = np.array([v.y for v in vectors])
    W = np.array([v.z for v in vectors])

    axs.quiver([0,0,0],[0,0,0],[0,0,0],U,V,W,color = color)

plotVector(ax,(x,y,z),'g')
plotVector(ax,(xstar,ystar,zstar),'r')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)
plt.show()

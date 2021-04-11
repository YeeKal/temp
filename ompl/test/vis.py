from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('path.txt')
fig = plt.figure()
#ax = fig.gca(projection='3d')
ax = fig.gca()
ax.plot(data[:,0],data[:,1],'.-')

#
r=0.25
a=0.5
b=0.5
theta = np.arange(0, 2*np.pi, 0.01)
x = a + r * np.cos(theta)
y = b + r * np.sin(theta)
ax.plot(x,y)
plt.show()
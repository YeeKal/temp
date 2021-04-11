from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('c_path.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax = fig.gca()
ax.plot(data[:,0],data[:,1],data[:,2],'.-')

#
# center and radius
center = [0,0,0]
radius = 0.98

# data
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
ax.plot_surface(x, y, z,  rstride=4, cstride=4, color='pink')
plt.show()
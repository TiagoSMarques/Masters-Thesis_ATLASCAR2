
# %%
# from sympy import init_printing, symbols
# from sympy import *

# %matplotlib notebook
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel('xlabel')
ax.set_ylabel('ylabel')
ax.set_zlabel('zlabel')
ax.set_ylim([-20, 20])
ax.set_zlim([0, 0.6])
# ax.view_init(azim=-90, elev=0)  # x-z
ax.view_init(azim=0, elev=90)  # x-y

# para a projecao ortogonal
# from mpl_toolkits.mplot3d import proj3d


# def orthogonal_proj(zfront, zback):
#     a = (zfront+zback)/(zfront-zback)
#     b = -2*(zfront*zback)/(zfront-zback)
#     return np.array([[1, 0, 0, 0],
#                      [0, 1, 0, 0],
#                      [0, 0, a, b],
#                      [0, 0, 0, zback]])


# proj3d.persp_transformation = orthogonal_proj

# 110 de amplitude com 0.5 de espacamento
start = 35.0/180*np.pi
stop = 145.0/180.0*np.pi

# mudar para 220 pontos para ter 0.5graus de amplitude
ph = np.linspace(start, stop, 10)
x = np.linspace(0, 30, 100)
n_vals = np.linspace(1, 4, 4)
x_last = []
y_last = []
z_last = []
h = 0.4
for n in n_vals:
    for phi in ph:
        alpha = (0.6+n*0.8)/180.0*np.pi  # 0.0105
        # phi = 0.6109
        l = h/np.tan(alpha)
        k = l/np.tan(phi)

        y = x*k/(2*l)
        z = -y*h/k+h
        ax.plot(x, y, z)
    x_last.append(y[-1])  # last element of array
    y_last.append(x[-1])  # last element of array
    z_last.append(z[-1])  # last element of array
# print(y)

plt.show()

# x_last.append(x[-1])  # last element of array
# y_last.append(y[-1])  # last element of array
# z_last.append(z[-1])  # last element of array

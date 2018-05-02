
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

ax.set_xlim([0, 45])
ax.set_ylim([-20, 20])
ax.set_zlim([0, 1.5])

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
# ax.view_init(azim=-90, elev=0)  # x-z
# ax.view_init(azim=0, elev=90)  # x-y

# 110 de amplitude com 0.5 de espacamento
start = 35.0/180*np.pi
stop = 145.0/180.0*np.pi

# mudar para 220 pontos para ter 0.5graus de amplitude
ph = np.linspace(start, stop, 22)
n_vals = np.linspace(1, 4, 4)
x_last = []
y_last = []
z_last = []
h = 0.4
sd_h = 0.15


# a plane is a*x+b*y+c*z+d=0
# [a,b,c] is the normal. Thus, we have to calculate
point = np.array([5, 10, 0])
normal = np.array([0, 1, 1])
d = -point.dot(normal)
xx, zz = np.meshgrid(np.linspace(0, 40, 2), np.linspace(0, sd_h, 2))
# calculate corresponding z
yy = 10

for n in n_vals:
    for phi in ph:
        x = np.linspace(0, 40, 100)
        alpha = (0.6+n*0.8-0.8)/180.0*np.pi  # 0.0105
        # l = h/np.tan(alpha)
        # k = l/np.tan(phi)

        # y = x*k/(2*l)
        y = x/(2*np.tan(phi))
        # z = -y*h/k+h
        z = -1*x/2*np.tan(alpha)+h

        prim_maior = np.argmax(y > yy)
        if prim_maior != 0 and z[prim_maior] <= sd_h:
            x = x[: prim_maior]
            y = y[: prim_maior]
            z = z[: prim_maior]

        prim_neg = np.argmax(z < 0.0)
        if prim_neg != 0:
            x = x[0: prim_neg]
            y = y[0: prim_neg]
            z = z[0: prim_neg]

        ax.plot(x, y, z)
        x_last.append(x[-1])  # last element of array
        y_last.append(y[-1])  # last element of array
        z_last.append(z[-1])  # last element of array
    ax.scatter(x_last, y_last, z_last, c="b")


ax.plot_surface(xx, yy, zz, color="y")

# draw sphere
centro = np.array([33, 0, 0])

u, v = np.mgrid[0: 2*np.pi: 20j, 0: np.pi: 10j]
r = 3
xs = r*np.cos(u)*np.sin(v)+centro[0]
ys = r*np.sin(u)*np.sin(v)+centro[1]
zs = r*np.cos(v)+0.2+centro[2]
# plt.axis('equal')
# ax.plot_wireframe(xs, ys, zs, color="r")

# draw the plane in the sphere

norm_plan = np.array([1, 1, 0.2])

# x_plan, z_plan = np.meshgrid(np.linspace(
#     centro[0]-5, centro[0]+5, 2), np.linspace(centro[2]-r/2, centro[2]+r/2, 2))

y_plan, z_plan = np.meshgrid(np.linspace(
    centro[1]-5, centro[1]+5, 10), np.linspace(centro[2]-r/2, centro[2]+r/2, 10))

x_plan = centro[0]-1/norm_plan[0] * \
    (norm_plan[1]*(y_plan-centro[1])+norm_plan[2]*(z_plan-centro[2]))

# y_plan = centro[1]-1/norm_plan[1] * \
#     (norm_plan[0]*(x_plan-centro[0])+norm_plan[2]*(z_plan-centro[2]))

# z_plan = centro[2]-1/norm_plan[2] * \
#     (norm_plan[1]*(y_plan-centro[1])+norm_plan[0]*(x_plan-centro[0]))

# ax.scatter(centro[0], centro[1], centro[2], c="g")
# ax.plot_surface(x_plan, y_plan, z_plan, rstride=1, cstride=1, alpha=0.6)

num_in_plane = 0
num_in_sphere = 0

# print(x_last)
# print("normal:", norm_plan, "centro:", centro)
# print("h:", h, "alpha:", alpha, "phi:", phi)

for elem in range(len(x_last)):
    # verifica se o ponto esta dentro da esfera
    point_sph = (x_last[elem]-centro[0])**2 + \
        (y_last[elem]-centro[1])**2+(z_last[elem]-centro[2])**2
    # se estiver
    if point_sph <= r**2:
        num_in_sphere += 1
        # verifica se o ponto pertence ao plano
        point_plain = (centro[0]-x_last[elem])*norm_plan[0]+(centro[1]-y_last[elem]) * \
            norm_plan[1]+(centro[2]-z_last[elem])*norm_plan[2]
        # se estiver
        if point_plain >= -0.5 and point_plain <= 0.5:
            num_in_plane += 1

print("num pontos esfera: ", num_in_sphere)
print("num pontos plano: ", num_in_plane)

plt.show()

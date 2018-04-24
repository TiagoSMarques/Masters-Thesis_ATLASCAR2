
#%%
from sympy import *
init_printing()
x = Symbol('x')
a = Integral(cos(x)*exp(x), x)
func = Eq(a, a.doit())
# para imprimir no ecra a forma simplificada
func
# simplify(func)
# para converter para latex
latex(func)

#%%
from sympy.abc import x, z
init_printing()
expr = x**2*z+1
expr
# este metodo é lento
expr.evalf(subs={z: 1, x: 4})
#%%
# mais rapido
f = lambdify([x, z], expr)
# f = lambdify([x, z], expr, "numpy")
f(4, 1)
#%%
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

x = np.linspace(0, 20, 100)
plt.plot(x, np.sin(x))
plt.show()

#%%
f = factorial
n = Symbol('n', integer=True)
R = 9801/sqrt(8)/Sum(f(4*n)*(1103+26390*n)/f(n)**4/396**(4*n), (n, 0, oo))
R
N(R, 10)

#------------------------------
#%%
from sympy import symbols
from numpy import linspace
from sympy import lambdify
import matplotlib.pyplot as mpl

t = symbols('t')
x = 0.05*t + 0.2/((t - 5)**2 + 2)
lam_x = lambdify(t, x, modules=['numpy'])

x_vals = linspace(0, 10, 100)
y_vals = lam_x(x_vals)

mpl.plot(x_vals, y_vals)
mpl.ylabel("Speed")
mpl.show()

#%%
# from sympy.abc import x, y, xp, yp, h, theta
init_printing()


#%% Função principal
x, y, xp, yp, n, h, theta, m, b = symbols('x y xp yp n h theta m b')
# Retas dos feixes laser
laser = tan(theta*n)*x + h
# Reta arbritraria (que pssará a plano)
plano = m*x + b

#%% Raio do circulo para depois filtrar
circle = (x - xp)**2 + (y-yp)**2
circle

# #%%
# #  81.
# x = symbols('x')
# y = Function('y')(x)
# eq = x**2 + 2*y**2 - 9
# f = sqrt((9 - x**2)/2)
# dydx = diff(eq, x)
# root = solve(dydx, diff(y, x, 1))

# # dy/dx = -x/2*y
# # the slope of tangent line at (1, 2)
# m_t = Rational(-1, 2*2)
# print(m_t)
# y1 = m_t*(x - 1) + 2

# # the slope of normal line at (1,2)
# y2 = (-1/m_t)*(x - 1) + 2
# eq, f, dydx, root, y1, y2

# yl = 4
# p = plot(f, -f, y1, y2, (x, -4, 4), ylim=(-yl, yl),
#          title="x**2 + 2*y**2 = 9", show=False)
# p[2].line_color = 'green'
# p[3].line_color = 'purple'
# p.show()

# #%% Teste para desenhar 4 linhas com paraemtros variaveis
# x, n = symbols('x n')
# y = -n*x*0.0139 + 0.4  # espaçamento de 0.8
# funcs = lambdify([n, x], y, modules=['numpy'])
# n_vals = np.linspace(1, 4, 4)
# for i in n_vals:
#     x_vals = np.linspace(0, 30)
#     y_vals = funcs(i, x_vals)
#     p1 = plt.plot(x_vals, y_vals)

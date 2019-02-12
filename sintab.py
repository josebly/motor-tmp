#!/usr/bin/env python
#%%
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

n = 512
nt = 10000
x = np.linspace(0, 2*(n-1)/n*np.pi, n)
xt = np.linspace(0, float(nt-1)/nt*2*np.pi, nt)
stab = np.sin(x)
ctab = np.cos(x)
plt.plot(x, np.sin(x))
plt.show() 

i = np.int_(np.floor(xt*n/2/np.pi))
(f,b) = np.modf(xt*n/2/np.pi)
ff = f*2*np.pi/n
sin_est1 = stab[i]
sin_est2 = stab[i] + ff*ctab[i]
sin_est3 = stab[i] + ff*ctab[i] - .5*ff*ff*stab[i]
sin_est4 = stab[i] + ff*ctab[i] - .5*ff*ff*stab[i] - 1.0/6.0*ff*ff*ff*ctab[i]
print(f)
plt.plot(xt, np.sin(xt)-sin_est1, xt, np.sin(xt)-sin_est2, xt, np.sin(xt)-sin_est3, xt, np.sin(xt)-sin_est4)
plt.show() 

plt.plot(xt, np.sin(xt)-sin_est4)
plt.show()
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

maxTime = 5
L = 0.8
g = 10
maxTime = 10
L = 0.8
dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
logTime = np.arange(0.0, maxTime, dt)

def rp(x, t):
    return [x[1], 
            -g/L*np.sin(x[0])]

theta = odeint(rp, [q0, 0], logTime)
logTheta = theta[:,0]

plt.grid(True)
plt.plot(logTime, logTheta, label = "theorPos")
plt.legend()

plt.show()

# 1 избавиться от затухания синуса
# 2 понять, откуда берется невязка между траекторией симулятора и моделью
# и попытаться ее минимизировать + посчитать нормы L2' и Linf
# есть как минимум два источника невязки
# идеал = l2(1.8766961224229702e-06), linf(4.531045163083669e-06)

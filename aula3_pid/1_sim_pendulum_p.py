#!/usr/bin/env python3
from time import sleep

import mujoco as mj
import mujoco.viewer
import numpy as np
from numpy import cos, pi, sin
import os
import matplotlib.pyplot as plt

model = mj.MjModel.from_xml_path("assets/pendulum.xml")
data = mj.MjData(model)

t = 0
TMAX = 1000

# data.qpos[:] = array([0, 0, 0, -pi/2, 0, 0, 0])

theta = []
time = []
thetad = 10 * np.pi/180 # 10 graus em radianos
# Kp = 25.36
Kp = 70

m = model.body_mass[1]
dim = model.geom_size[1]
a = dim[0]
b = dim[2]

print("body mass = ", m)
# print("body mass = ", dim)
print("body inertia = ", 1/12*m*(a**2+4*b**2))

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        # meu primeiro controlador de posição
        data.ctrl[0] = Kp*(thetad - data.qpos[0])

        mj.mj_step(model, data)
        time.append(data.time)
        theta.append(data.qpos[0])

        viewer.sync()
        t += 1
        if t > TMAX:
            break
        sleep(0.01)



plt.plot(time, [ti*180/np.pi for ti in theta])
plt.plot(time, np.ones_like(theta)*thetad*180/np.pi, '--')
plt.xlabel("Tempo (s)")
plt.ylabel("Ângulo (deg)")
plt.legend(["q1", "qd"])
plt.grid()
plt.show()
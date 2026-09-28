#!/usr/bin/env python3
from time import sleep

import mujoco as mj
import mujoco.viewer
import numpy as np
from numpy import cos, pi, sin
import os
import matplotlib.pyplot as plt

model = mj.MjModel.from_xml_path("assets/cube.xml")
data = mj.MjData(model)

t = 0

# data.qpos[:] = array([0, 0, 0, -pi/2, 0, 0, 0])

x = []
x1 = []

xd = 0.5
Kp = 10
Kd = 2

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        # distúrbio
        # frc_disturbio = 0.1*cos(0.01*t) # Nm
        
        # meu primeiro controlador de posição
        data.ctrl[0] = Kp*(xd - data.qpos[0]) + Kd*(-data.qvel[0]) # + frc_disturbio# N

        mj.mj_step(model, data)
        x.append(data.qpos[0])

        viewer.sync()
        t += 1
        if t > 1000:  # and os.getenv('TESTING') is not None:
            break
        sleep(0.01)

plt.plot(x)
plt.plot(np.ones_like(x)*xd, '--')
plt.legend(["q1", "qd"])
plt.grid()
plt.show()
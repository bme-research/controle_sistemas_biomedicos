import mujoco as mj
import mujoco.viewer
import time
import matplotlib.pyplot as plt
import numpy as np


model = mj.MjModel.from_xml_path("assets/cube.xml")
data = mj.MjData(model)

# increase joint friction
# model.jnt[0].frictionloss = 0.1

force_duration = 0.5
start_time = data.time
T_SIM_MAX = 10

time_history = []
qpos_history = []

xd = 0.5
K = 20
B = 5

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time - start_time <= T_SIM_MAX:
    
            x = data.qpos[0]
    
            frc_aplicada = -K*(x - xd) - B*data.qvel[0]
    
            data.qfrc_applied[0] = frc_aplicada
    
            time_history.append(data.time - start_time)
            qpos_history.append(data.qpos.copy())
    
            # visualize_force(viewer, data.xpos[body_id], [data.qfrc_applied[0], 0, 0])
    
            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)
            if data.time - start_time > T_SIM_MAX:
                break
    
    qpos_history = np.array(qpos_history)
    time_history = np.array(time_history)
    for idx in range(qpos_history.shape[1]):
        plt.plot(time_history, qpos_history[:, idx], label=f"qpos[{idx}]")
    plt.plot(time_history, np.ones_like(time_history)*xd, 'r--', label='xd')
    plt.xlabel("Time [s]")
    plt.ylabel("Joint position")
    plt.title("qpos over time")
    plt.legend()
    plt.grid(True)
    plt.show()
    
    
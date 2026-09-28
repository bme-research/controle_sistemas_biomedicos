import mujoco as mj
import mujoco.viewer
import numpy as np
import time

model = mj.MjModel.from_xml_path("assets/cube_mbk.xml")
data = mj.MjData(model)

target_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "q_target")
target_qpos_adr = model.jnt_qposadr[target_joint_id]
target_dof_adr = model.jnt_dofadr[target_joint_id]
target_actuator_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "target_position")

# Prescribe a sinusoidal position (kinematic restriction) on the massless target body
freq_hz = 0.5
amplitude = 0.025  # m

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        pos = amplitude * np.sin(2 * np.pi * freq_hz * data.time)
        vel = amplitude * 2 * np.pi * freq_hz * np.cos(2 * np.pi * freq_hz * data.time)

        # rigidly force qpos/qvel to the prescribed trajectory, bypassing the
        # joint's own dynamics; ctrl is kept aligned so the position actuator
        # (kp=1000) contributes ~0 extra force instead of fighting the override
        data.qpos[target_qpos_adr] = pos
        data.qvel[target_dof_adr] = vel
        data.ctrl[target_actuator_id] = pos

        mj.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

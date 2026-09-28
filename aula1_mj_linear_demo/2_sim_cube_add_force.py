import time
import numpy as np
import matplotlib.pyplot as plt
import mujoco as mj
import mujoco.viewer


def visualize_force(viewer, origin, force, scale=0.05, width=0.006,
                     rgba=(1.0, 0.0, 0.0, 1.0), min_force=1e-6):
    """Draw (or clear) an arrow in the viewer representing a force vector.

    origin: (3,) world-frame point the force is applied at, e.g. data.xpos[body_id].
    force: (3,) world-frame force vector in N. Arrow length/width scale with its magnitude.
    Call once per step; passing a ~zero force clears the arrow instead of drawing one.
    """
    scn = viewer.user_scn
    scn.ngeom = 0

    force = np.asarray(force, dtype=float)
    mag = np.linalg.norm(force)
    if mag < min_force:
        return

    origin = np.asarray(origin, dtype=float)
    tip = origin + force * scale

    geom = scn.geoms[0]
    mj.mjv_initGeom(
        geom,
        type=mj.mjtGeom.mjGEOM_ARROW,
        size=np.zeros(3),
        pos=np.zeros(3),
        mat=np.eye(3).flatten(),
        rgba=np.array(rgba, dtype=np.float32),
    )
    geom.label = f"{mag:.1f} N"
    mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_ARROW, width, origin, tip)
    scn.ngeom = 1


model = mj.MjModel.from_xml_path("assets/cube.xml")
data = mj.MjData(model)
body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link1")

force_duration = 0.5
start_time = data.time
T_SIM_MAX = 10

time_history = []
qpos_history = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time - start_time <= T_SIM_MAX:

        x = data.qpos[0]

        # if x < 0.25:
        #     frc_aplicada = 1
        # elif x >= 0.25 and x < 0.4975:
        #     frc_aplicada = -1
        # else:
        #     frc_aplicada = 0

        if data.time - start_time == 0.01:
            frc_aplicada = 100
        else:
            frc_aplicada = 0

        if abs(x - 0.5) < 0.01:
            frc_aplicada = -1

        data.qfrc_applied[0] = frc_aplicada

        time_history.append(data.time - start_time)
        qpos_history.append(data.qpos.copy())

        visualize_force(viewer, data.xpos[body_id], [data.qfrc_applied[0], 0, 0])

        mj.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
        if data.time - start_time > T_SIM_MAX:
            break

qpos_history = np.array(qpos_history)
time_history = np.array(time_history)
for idx in range(qpos_history.shape[1]):
    plt.plot(time_history, qpos_history[:, idx], label=f"qpos[{idx}]")
plt.xlabel("Time [s]")
plt.ylabel("Joint position")
plt.title("qpos over time")
plt.legend()
plt.grid(True)
plt.show()


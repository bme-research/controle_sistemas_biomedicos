import mujoco as mj
import mujoco.viewer
import numpy as np
import time

model = mj.MjModel.from_xml_path("assets/cube_mbk.xml")
data = mj.MjData(model)
body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "target_massless")

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

# Apply a sinusoidal generalized force to the massless target body
target_joint_id = 1   # q_target is the second joint in the XML
freq_hz = 0.5
amplitude = 3

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        force = amplitude * np.sin(2 * np.pi * freq_hz * data.time)

        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[target_joint_id] = force

        visualize_force(viewer, data.xpos[body_id], [data.qfrc_applied[target_joint_id], 0, 0])

        mj.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

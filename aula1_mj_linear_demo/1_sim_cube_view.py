import time
import mujoco as mj
import mujoco.viewer

model = mj.MjModel.from_xml_path("assets/cube.xml")
data = mj.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:

        mj.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

import numpy as np
import time
import mujoco
import mujoco.viewer
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE) # SO-101 scene.xml
data = mujoco.MjData(model)

data.ctrl[:5] = np.radians([0, 30, -45, 0, 0])  # 5축 목표각을 건다

with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running:
        mujoco.mj_step(model, data)
        v.sync()
        time.sleep(model.opt.timestep)
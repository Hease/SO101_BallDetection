import numpy as np
import time
import mujoco
import mujoco.viewer
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE) # SO-101 scene.xml
data = mujoco.MjData(model)

data.qpos[:5] = np.radians([10, 30, -45, 0, 0])  # 5축 목표각을 건다
mujoco.mj_forward(model, data)

eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
print("손끝 위치[m]: ", np.round(data.site_xpos[eid], 3))

import numpy as np
import mujoco
import mujoco.viewer
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE) # SO-101 scene.xml
data = mujoco.MjData(model)

data.qpos[:5] = np.radians([0, 30, -45, 0, 0])
data.ctrl[:5] = np.radians([0, 30, -45, 0, 0])  # 5축 목표각(도->라디안), 초기자세
mujoco.mj_forward(model, data)  # 그 각도로 위치 계산

mujoco.viewer.launch(model, data)   # 팔 모양 확인
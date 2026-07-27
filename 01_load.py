import mujoco
import mujoco.viewer
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE) # SO-101 scene.xml
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)   # 팔 모양 확인
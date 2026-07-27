from soarm_lab.grasp_scene import build, BALL_XY, BALL_R
import numpy as np, mujoco, mujoco.viewer

# arm.live()
# arm.go([0.25, 0.0, 0.15])
# arm.go([0.20, 0.12, 0.10], grip=90) # 좌표 + 그리퍼 90도
# arm.wait()  # 창을 열어둔 채 대기

model, data = build(basket_xy=(0.16, -0.12))    # 공 + 바구니 씬
bid = model.body("obj").id
print("공 위치: ", data.xpos[bid])

mujoco.viewer.launch(model, data)

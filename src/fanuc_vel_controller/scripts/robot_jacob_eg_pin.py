#!/usr/bin/env python3

import pinocchio
import numpy as np

model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
data = pinocchio.createDatas(model)[0]

# print(pinocchio.randomConfiguration(model))
qz = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)

eeFrameId = model.getFrameId("tool0")

# end effector - jacobian
jac = pinocchio.computeFrameJacobian(model, data, qz, eeFrameId)
print(np.round(jac, 4))

tempVel = np.array([0, 0, 1, 0, 0, 0]).reshape((6,1))
print(np.round(np.linalg.pinv(jac) @ tempVel, 4))
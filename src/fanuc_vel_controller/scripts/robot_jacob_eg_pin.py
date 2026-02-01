#!/usr/bin/env python3

import pinocchio
import numpy as np
from pinocchio.visualize import MeshcatVisualizer

model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
data = pinocchio.createDatas(model)[0]

# print(pinocchio.randomConfiguration(model))
qz = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)

eeFrameId = model.getFrameId("tool0")

### EE jacobian example
"""
# end effector - jacobian
jac = pinocchio.computeFrameJacobian(model, data, qz, eeFrameId)
print(np.round(jac, 4))

tempVel = np.array([0, 0, 1, 0, 0, 0]).reshape((6,1))
print(np.round(np.linalg.pinv(jac) @ tempVel, 4))
"""

### visualization example
"""
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)

viz.loadViewerModel()

q0 = pinocchio.neutral(model)
q_rand = pinocchio.randomConfiguration(model)
viz.display(q_rand)
viz.displayVisuals(True)

while True:
    pass
"""

# collision avoidance
collision_model.addAllCollisionPairs()
geom_data = pinocchio.GeometryData(collision_model)
q = np.array([0.0, 10.0, 0.0, 0.0, 0.0, 0.0])

pinocchio.computeCollisions(model, data, collision_model, geom_data, q, False)

# Print the status of collision for all collision pairs
for k in range(len(collision_model.collisionPairs)):
    cr = geom_data.collisionResults[k]
    cp = collision_model.collisionPairs[k]
    print(
        "collision pair:",
        cp.first,
        ",",
        cp.second,
        "- collision:",
        "Yes" if cr.isCollision() else "No",
    )
    name1 = collision_model.geometryObjects[cp.first].name
    name2 = collision_model.geometryObjects[cp.second].name
    print(f"Name of paris: {name1} - {name2}\n")
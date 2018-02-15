This directory contains files that provide handcoded information to the robot:

* Grasps (how have I grasped objects)
* Object models (what is in my hands)
* Object info (edge, tip, etc.)

Ideally this should not be needed at all, because the robot should be able
to infer or recognize all such data about it's environment.

Note:
When object info is given then feature_detector will be bypassed.


Sample file:

```
# Object scans a.k.a. object knowledge base
point-clouds:
  tool: b_table_knife.ply
  target-object: b_bucket.ply

tool-mass: 0.050

# Transformation from the end effector to the tool
tool-grasp: 
  frame:
    - quaternion: [0.723185,0,0,0.690655] # x, y, z, w
    - vector3: [0.060878,-0.002438,0.005864] # x, y, z

# Transformation from the end effector to the target object
target-object-grasp:
  frame: 
    - quaternion: [-0.0216269,-0.756025,-0.121089,-0.642881]
    - vector3: [0.0577053,0.0189525,0.101375]

Note: when this info is given then edge_detector won't be invoked
object-info:
  edge-point: {vector3: [0, 0.090856, -0.054262]}
  alignment-vector: {vector3: [0, -0.088015, -0.008336]}
  grasp-center: {vector3: [-0.058807, 0.00048464, 2.9943e-05]}
  action-center: {vector3: [0.045684, 0.011433, 0.00074432]}
  tool-tip: {vector3: [0.10683, 0.0018891, 0.0015357]}
  tool-tip-vector: {vector3: [0.16564, 0.0014045, 0.0015058]}
  tool-heel: {vector3: [-0.005198, 0.00063569, 0.00016369]}
  tool-quaternion: {quaternion: [0.70381, 0.005861, 0.066979, -0.7072]}
```
import math

#Kinematic parameters matching the actual PUMA560.ttt CoppeliaSim scene.
#Unlike robot_params.py (textbook/reference DH values), these were reverse-engineered
#from the scene itself: PUMA560.ttt was built by importing the open-source
#puma560_description URDF package, whose joint geometry does not match the textbook
#DH table (e.g. joint 1 rotates about local Y, not Z, so it isn't expressible as a
#plain per-joint alpha/a/d/theta row). Values below are the URDF's joint origins,
#verified against the live scene to sub-millimeter precision (see fw_kin_puma560_scene.py).

num_joints = 6

#Per joint: fixed origin translation (xyz) [m], fixed origin rotation (roll, pitch, yaw) [rad],
#and the joint's own rotation axis, all expressed in the parent joint's frame.
#theta_i (the commanded joint angle) rotates about that joint's axis, applied *after* the
#fixed origin transform.
joint_origin_xyz = [
    (0.0,    0.0,     0.6718),
    (0.0,    0.0,     0.0),
    (0.4318, -0.0203, 0.1501),
    (0.0,    0.0,     0.0),
    (0.0,    0.0,     0.4331),
    (0.0,    0.0558,  0.0),
]

joint_origin_rpy = [
    (math.pi / 2, 0.0, 0.0),
    (0.0,         0.0, 0.0),
    (0.0,         0.0, -math.pi / 2),
    (math.pi / 2, 0.0, math.pi / 2),
    (math.pi / 2, 0.0, 0.0),
    (-math.pi / 2, 0.0, 0.0),
]

joint_axis = [
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
]

#Fixed offset from joint 6 to the tip (last visual link's origin), no rotation.
tip_offset_xyz = (0.0, 0.0, -0.010)

#Fixed correction between the scene's base object frame ("link1_visual", the root
#object CoppeliaSim exposes) and the true kinematic root the joint-origin chain above
#is built from. The URDF's link1 visual mesh carries its own "origin rpy 0 0 pi/2"
#for mesh alignment, and CoppeliaSim's importer bakes that into the base object's own
#frame -- so the scene's root object is rotated +90 deg about Z relative to the
#kinematic root. Confirmed empirically: without this correction, driving joint 1 to
#90 deg in the simulator reproduces exactly the pose this chain computes for 0 deg.
base_correction_rpy = (0.0, 0.0, -math.pi / 2)

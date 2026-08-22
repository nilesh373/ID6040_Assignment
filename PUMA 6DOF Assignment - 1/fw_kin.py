import math
import numpy as np
import robot_params

#PUMA 560 forward kinematics -- 6-DOF serial manipulator
#=========================================================
#Complete the three functions below. See robot_params.py for what each joint's
#joint_origin_xyz[i], joint_origin_rpy[i] (fixed) and joint_axis[i] (rotation axis for
#joint_angles[i]) mean, plus tip_offset_xyz and base_correction_rpy.

#The helpers below are provided -- build each transform as a 4x4 homogeneous matrix
#(_T_from_Rp) using a rotation (_rpy_to_R or _rot_about_axis) and a translation.

def _rpy_to_R(roll, pitch, yaw):
    #Fixed-axis (extrinsic) roll-pitch-yaw, matching URDF's <origin rpy="r p y"> convention
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def _T_from_Rp(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def _rot_about_axis(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(theta), math.sin(theta), 1 - math.cos(theta)
    R = np.array([
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c]
    ])
    return _T_from_Rp(R, (0.0, 0.0, 0.0))

def compute_T_i_im1(i, theta_i):
    #1) Calculate T^i_{i-1}: joint i's fixed origin transform (joint_origin_xyz/rpy[i])
    #   followed by the variable rotation theta_i [rad] about joint_axis[i].

    ### Fill this part ###
    T_i_im1 = ...

    return T_i_im1

def compute_T_0_n(joint_angles):
    #2) Calculate T^0_n: chain base_correction_rpy, then compute_T_i_im1(i, theta_i) for
    #   each joint i = 0..num_joints-1 (theta_i = joint_angles[i] converted to radians),
    #   then the final tip_offset_xyz.

    ### Fill this part ###
    T_0_n = ...

    return T_0_n

def fw_kin(joint_angles):
    #3) Calculate the Cartesian space coordinates of the end effector -- the position
    #   vector and the orientation, relative to the base -- from T_0_n = compute_T_0_n(...).
    #   Position is the translation column of T_0_n; orientation is its top-left 3x3
    #   rotation block.

    ### Fill this part ###
    T_0_n = ...
    end_effector_position_analytic = ...
    
    return end_effector_position_analytic, T_0_n

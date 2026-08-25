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
    R_origin = _rpy_to_R(*robot_params.joint_origin_rpy[i]) # Convert roll, pitch, yaw to rotation matrix
    p_origin = robot_params.joint_origin_xyz[i] # Get the translation vector for the joint origin
    T_origin = _T_from_Rp(R_origin, p_origin) # Build the homogeneous transformation matrix for the joint origin
    T_joint = _rot_about_axis(robot_params.joint_axis[i], theta_i) # Build the homogeneous transformation matrix for the joint rotation
    return T_origin @ T_joint # Return the combined transformation matrix from joint i-1 to joint i

def compute_T_0_n(joint_angles):
    # Convert joint angles to radians
    joint_angles_rad = [math.radians(angle) for angle in joint_angles] 

    T_0_n = _T_from_Rp(_rpy_to_R(*robot_params.base_correction_rpy), (0.0, 0.0, 0.0)) # Initialize T_0_n with the base correction transformation

    for i in range(robot_params.num_joints):
        T_0_n = T_0_n @ compute_T_i_im1(i, joint_angles_rad[i]) # Multiply the current transformation matrix by the transformation from joint i-1 to joint i    

    T_tip = _T_from_Rp(_rpy_to_R(0.0, 0.0, 0.0), robot_params.tip_offset_xyz) # Build the homogeneous transformation matrix for the tip offset
    T_0_n = T_0_n @ T_tip # Multiply the current transformation matrix by the tip offset transformation to get the final transformation matrix from the base to the end effector

    return T_0_n

def fw_kin(joint_angles):
    T_0_n = compute_T_0_n(joint_angles) # Compute the transformation matrix from the base to the end effector using the provided joint angles
    end_effector_position_analytic = T_0_n[:3, 3] # Extract the position of the end effector from the transformation matrix
    
    return end_effector_position_analytic, T_0_n # Return both the position of the end effector and the full transformation matrix from the base to the end effector

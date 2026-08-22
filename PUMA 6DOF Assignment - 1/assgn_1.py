#!/usr/bin/env python

"""
Manipulator simulation setup
@author: Vrijesh Kunwar
"""

#Import libraries
import time
import math
import random
import numpy as np
#Import files
import sim_interface
import fw_kin


def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):

            #Exercise: Forward kinematics

            #Get random joint angles
            joint_angles = [random.randint(-90, 90) for _ in range(sim_interface.NUM_JOINTS)] #In degrees
            print("joint angles (commanded)", joint_angles)

            #Set joint angles
            sim_interface.set_joint_position(joint_angles)
            time.sleep(0.5)

            #Joints may clamp at their limits, so read back what was actually reached
            #and use that for the analytic comparison rather than the commanded angles
            joint_angles_reached = [math.degrees(angle) for angle in sim_interface.get_joint_position()]
            print("joint angles (reached)", joint_angles_reached)

            #Obtain end effector pose (position + orientation) from simulation
            end_effector_position_sim, end_effector_orientation_sim = sim_interface.get_end_effector_pose()

            #Compute end effector pose analytically:
            #  1) T^i_{i-1} for each joint (fw_kin.compute_T_i_im1)
            #  2) T^0_n, the manipulator transformation matrix (fw_kin.compute_T_0_n)
            #  3) the position vector and orientation of the end effector, relative to the base
            end_effector_position_analytic, T_0_n = fw_kin.fw_kin(joint_angles_reached)
            end_effector_orientation_analytic = T_0_n[:3, :3]

            print("End effector position (analytic)", end_effector_position_analytic)
            print("End effector orientation (analytic)", end_effector_orientation_analytic.tolist())

            #Verify position and orientation independently
            position_error = sum(math.fabs(a - b) for a, b in zip(end_effector_position_analytic, end_effector_position_sim))
            orientation_error = np.abs(end_effector_orientation_analytic - np.array(end_effector_orientation_sim)).sum()

            print("Position error (sum of abs differences):", position_error)
            print("Orientation error (sum of abs differences):", orientation_error)

            if position_error < 0.01 and orientation_error < 0.01:
                print("Exercise result: Success")
            else:
                print("Exercise result: Failed")

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')

    #Stop simulation
    sim_interface.sim_shutdown()
    time.sleep(0.5)
    return

#run
if __name__ == '__main__':

    main()
    print ('Program ended')
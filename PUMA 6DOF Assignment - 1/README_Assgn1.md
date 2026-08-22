# System Overview
This project validates the forward kinematics of a 6-DOF PUMA 560 manipulator by comparing analytically computed poses against a live CoppeliaSim simulation. The system relies on a Python environment communicating with the simulator via a ZeroMQ remote API.

## File Inventory and Architecture
Before execution, ensure all files are saved in the same working directory with the appropriate filenames:

**`PUMA560.ttt`:** The CoppeliaSim binary scene file containing the 3D environment and the PUMA 560 robot model. 
**`robot_params.py`:** Contains the reverse-engineered kinematic parameters (joint origins, axes, and base corrections) that exactly match the physical URDF geometry of the CoppeliaSim scene.
**`fw_kin.py`:** The mathematical module that imports `robot_params.py` to analytically compute the forward kinematics and transformation matrices for the manipulator.
**`sim_interface.py`:** The communication wrapper that uses the `coppeliasim_zmqremoteapi_client` to initialize the server, grab object handles, control joint positions, and extract positional data from the simulator.
**`assgn_1.py`:** The primary execution script that imports `sim_interface` and `fw_kin` to orchestrate the entire testing process.

## Execution Sequence
To successfully run the simulation and kinematic comparison, follow this exact sequence:
1. Read all the files carefully, understand the code.Then make neccesary changes `only in file fw_kin.py`
2.  **Launch the Simulator:** Open the CoppeliaSim application and load the `PUMA560.ttt` scene file and start the simulation. 
3.  **Verify API Server:** Ensure CoppeliaSim's built-in ZeroMQ remote API server is running (it listens on `localhost:23000` by default).
4.  **Run the Main Script:** Execute `assgn_1.py` in your Python environment. 


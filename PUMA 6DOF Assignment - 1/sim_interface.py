import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = None

NUM_JOINTS = 6
JOINT_NAMES = ["/j1", "/j2", "/j3", "/j4", "/j5", "/j6"]
BASE_NAME = "/link1_visual"
END_EFFECTOR_NAME = "/link7_visual"

def sim_init():
  global sim

  #Connect to CoppeliaSim's ZeroMQ remote API server (built in, listens on localhost:23000 by default)
  try:
    client = RemoteAPIClient()
    sim = client.require('sim')
    print ('Connected to CoppeliaSim ZMQ remote API server')
    return True
  except Exception as e:
    print ('Failed connecting to CoppeliaSim ZMQ remote API server:', e)
    return False

def get_handles():
  #Get the handles to the sim items

  global joint_handles
  global base_handle
  global end_effector_handle

  joint_handles = [sim.getObject(name) for name in JOINT_NAMES]
  base_handle = sim.getObject(BASE_NAME)
  end_effector_handle = sim.getObject(END_EFFECTOR_NAME)

  # Set all joints to zero to ensure manipulator is starting configuration
  for handle in joint_handles:
    sim.setJointPosition(handle, 0)

  print ("Succesfully obtained handles")

  return

def start_simulation():
  #Start the Simulation: Keep printing out status messages!!!
  sim.startSimulation()
  print ("---!!! Started Simulation !!! ---")
  return True

def get_joint_position():
  #Function that will return the current joint angles
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS
  joint_positions = [sim.getJointPosition(handle) for handle in joint_handles]

  print("current joint positions", joint_positions)

  return joint_positions

def set_joint_position(joint_angles):
  #Function to set the joint angles [in degrees]
  joint_angles_in_rad = [math.radians(angle) for angle in joint_angles]

  for handle, angle_rad in zip(joint_handles, joint_angles_in_rad):
    sim.setJointPosition(handle, angle_rad)

  return

def get_end_effector_position():
  #Function that will return the end effector position [in meters], relative to the robot's base frame
  end_effector_position = sim.getObjectPosition(end_effector_handle, base_handle)

  print("End effector position (simulation)", end_effector_position)
  return end_effector_position

def get_end_effector_pose():
  #Function that will return the end effector's position [in meters] and orientation
  #(3x3 rotation matrix), both relative to the robot's base frame
  matrix = sim.getObjectMatrix(end_effector_handle, base_handle)  # 12 floats, row-major [R|p]

  end_effector_position = [matrix[3], matrix[7], matrix[11]]
  end_effector_orientation = [matrix[0:3], matrix[4:7], matrix[8:11]]

  print("End effector position (simulation)", end_effector_position)
  print("End effector orientation (simulation)", end_effector_orientation)
  return end_effector_position, end_effector_orientation

def sim_shutdown():
  #Gracefully shutdown simulation
  sim.stopSimulation()
  print ("---!!! Stopped Simulation !!! ---")
  return

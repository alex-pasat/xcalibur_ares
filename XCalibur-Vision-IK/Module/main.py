import common
from profileExtraction import profileExtraction
from homography import homography
from postProcessing import knifeGeo
from IK import ik, robot
from yaw import process_yaw
from actuator_processing import velocity_ratios

import numpy as np
from math import pi

import time
import sys
start = time.time()
benchmarking = False

'''****************************************           CONSTANTS           *****************************************'''
path = "../bigPaper.jpg"
global_offset = np.array([109.5-52,145.97+19,131.5], dtype=float)  # This is measured from a homed position
bevel_angle = 15                                                    # in degrees one-sided
q0 = [[],[0,0,pi/2,pi/2,0],[robot.links[0].qlim[1],0,pi/2,-pi/2,0]] # index 1 and 2 for q1, q2 resp.

# Function Calls
# Wait for USB prompt
# Need to make somthing to take image and assign path

'''****************************************       PROFILE EXTRACTION      ****************************************'''
blade_profile = profileExtraction(path, debug=False)               # pixels uncorrected
if benchmarking: profile_extraction_time = time.time() - start
relative_profile = homography(path, blade_profile, debug=False)    # in mm relative to corner of checkers
profile, normals = knifeGeo(relative_profile, bevel_angle)         # compute normals vectors and switch to global coords
profile = (profile + global_offset)                                # locate within global coords
if benchmarking: homography_time = time.time() - start - profile_extraction_time

'''****************************************       KINEMATICS SIDE I      ****************************************'''
q1 = ik(robot, profile, normals, q0[1], debug=False)               # compute first side joint angles
q1 = process_yaw(q1, True)                                         # ensure yaw monotonic and segment profile
ratios1 = velocity_ratios(q1)                                      # calculate the velocity ratios

'''****************************************       KINEMATICS SIDE II     ****************************************'''
q2 = ik(robot, profile, common.flipZ(normals), q0[2], debug=False) # compute first side joint angles
q2 = process_yaw(q2, True)                                         # ensure yaw monotonic and segment profile
ratios2 = velocity_ratios(q2)                                      # calculate the velocity ratios
if benchmarking: kinematics_processing_time = time.time() - start - homography_time 

'''****************************************       PREPARE OUTPUTS        ****************************************'''
tip_q1 = q1[0]                                                     # joint variables to reach knife tip on first side
tip_q2 = q2[0]                                                     # joint variables to reach knife tip on second side
yaw_indices = q1[:,2]                                              # yaw values are same on both sides
output_array = [tip_q1, tip_q2, yaw_indices, ratios1, ratios2]     # all outputs to the stm
total_time = time.time() - start

if benchmarking: 
    print("profile_extraction_time = ",profile_extraction_time)
    print("homography_time = ",homography_time)
    print("kinematics_processing_time = ",kinematics_processing_time)
    print("total_time = ",total_time)
    total_bytes = sum(
        arr.nbytes if isinstance(arr, np.ndarray) else sys.getsizeof(arr)
        for arr in [tip_q1, tip_q2, yaw_indices, ratios1, ratios2]
    )
    print(f"output size = {total_bytes} bytes")

np.savez("knife_data.npz",
     tip_q1=tip_q1,
     tip_q2=tip_q2,
     yaw_indices=yaw_indices,
     ratios1=ratios1,
     ratios2=ratios2,
)

# Output over USB

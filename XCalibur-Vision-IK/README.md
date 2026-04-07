#######################   Procedure (currently implemented)   ###############################
1) Given an image, extract the blade profile
2) Warp the profile using a homography based on reference points on checkerboard
3) Scale to dimensions in mm from pixel arrays based on reference points
4) Downsample points and infer normal vectors based on profile and bevel angle
5) Apply iterative methods to solve joint variables to achieve tangency at each sample point
6) Approximate velocity ratios between actuators relative to yaw using finite differences
7) Create an output array containing only essential information

######################   Inputs (of planned implementation)   ###############################
1) Prompt to detect if knife inserted

1) Prompt to detect and solve knife geometry
2) Bevel angle

######################   Outputs (of planned implementation)   ###############################
1) Knife detected in enclosure or not

1) Array of joint variables to place whetstone at tip of knife for each side of knife
2) An array of yaw angles for each sample point order from tip to hilt
3) Arrays of ratios between velocity of actuators versus yaw velocity for each side

######################    Coordinate System      ############################################
Local coordinate systems are defined using DH convention relative to the global coordinate system
Global z-axis points along underpass axis in the direction from camera to clamp
Global x-axis points vertically upwards
Global y-axis points to the right from the point of view of the camera

I can add further to this with some images if necessary

######################    Interpretation of Outputs    #####################################
Below is an example from code in one of the kinematics verification scripts using
the outputs. It uses the outputs of the raspberry pi to animate the robot moving along one
side of the blade. Relevant lines begin with an arrow "->". A description of this process
in plain text will follow as well:


env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
env.add(knife)

q0 = [0,0,pi/2,pi/2,0]
robot.q = q0

-> data = np.load("knife_data.npz")
-> tip_q1 = data['tip_q1']           # joint variables to reach knife tip on first side
-> tip_q1 = data['tip_q1']           # joint variables to reach knife tip on second side
-> yaw_indices = data['yaw_indices'] # yaw values of sample points (note they are descending)
-> ratios1 = data['ratios1']         # ratio between actuator velocities and yaw velocity forfirst side
-> ratios2 = data['ratios2']         # ratio between actuator velocities and yaw velocity forsecond side

q0 = tip_q1
r = robot.fkine(q0).t
r = mm_to_m_vec(r)

shapes = build_shapes(q0,r,robot)
for s in shapes:
    env.add(s)

# ratio slice is an array with 4 elements for the specific yaw interval
-> def apply_ratio(yaw_v, ratio_slice):
->     velocity = yaw_v * ratio_slice # multiply other actuator velocities by appropriate ratios
->     velocity = [*velocity[:2], *[yaw_v], *velocity[2:]] # middle index is just the yaw velocity
->     return np.array(velocity)  

-> yaw_v = -pi/4 # set yaw velocity rad/s. Likely won't be constant irl
dt = 0.005

while True:
    # start at hilt
    robot.q = q0
    yaw_idx = 0

    # while the yaw hasn't reached the hilt
    while robot.q[2] > yaw_indices[-1]:
->        # check if we got to next yaw index
->        if robot.q[2] <= yaw_indices[yaw_idx+1]:
->            yaw_idx += 1
->            
->        # with correct idx, get velocity and update
->        velocity = apply_ratio(yaw_v, ratios1[yaw_idx])
->        qd = velocity
->
->        robot.q = robot.q + dt*qd

        pose = robot.fkine(robot.q)
        update_shapes(shapes, robot.q, pose.t)
        env.step(dt)

    sleep(0.5)

Here is a plain description of the procedure
Procedure:
1) If the yaw value is more than the value at the next yaw index, increase the yaw index
2) Choose a yaw velocity
3) Multiply by the corresponding ratios for the yaw index to calculate joint velocities

###################################    Benchmarks    #####################################
Running this on my laptop I get (units are seconds and bytes):

profile_extraction_time =  2.39435076713562
kinematics_processing_time =  0.7474410533905029
total_time =  3.141791820526123
output size = 4272 bytes


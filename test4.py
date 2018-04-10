import forwardKinematics
import vrep
import time
import numpy as np
#import hw515.py
import math


def distance_between_points(p_1, p_2):
    return math.sqrt((p_1[0] - p_2[0])**2 + (p_1[1] - p_2[1])**2 + (p_1[2] - p_2[2])**2)

def are_spheres_colliding(p_1, p_2, r_1, r_2):
    return distance_between_points(p_1, p_2) <= (r_1 + r_2)



# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the second joint of robot
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')

# Get "handle" to the third joint of robot
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')

# Get "handle" to the fourth joint of robot
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')

# Get "handle" to the fifth joint of robot
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')

# Get "handle" to the sixth joint of robot
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')


# Get "handle" to the zeroth dummy
result, Dummy0_handle = vrep.simxGetObjectHandle(clientID, 'Dummy0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for zeroth dummy')
    
# Get "handle" to the first dummy
result, Dummy1_handle = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first dummy')

# Get "handle" to the second dummy
result, Dummy2_handle = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second dummy')

# Get "handle" to the third dummy
result, Dummy3_handle = vrep.simxGetObjectHandle(clientID, 'Dummy3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third dummy')

# Get "handle" to the fourth dummy
result, Dummy4_handle = vrep.simxGetObjectHandle(clientID, 'Dummy4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth dummy')
"""
# Get "handle" to the fifth dummy
result, Dummy5_handle = vrep.simxGetObjectHandle(clientID, 'Dummy5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth dummy')
"""

p0 = vrep.simxGetObjectPosition(clientID, Dummy0_handle, -1, vrep.simx_opmode_blocking)
p0 = np.array(p0[1]).transpose()
p1 = vrep.simxGetObjectPosition(clientID, Dummy1_handle, -1,vrep.simx_opmode_blocking)
p1 = np.array(p1[1]).transpose()
p2 = vrep.simxGetObjectPosition(clientID, Dummy2_handle,-1,vrep.simx_opmode_blocking)
p2 = np.array(p2[1]).transpose()
p3 = vrep.simxGetObjectPosition(clientID, Dummy3_handle,-1, vrep.simx_opmode_blocking)
p3 = np.array(p3[1]).transpose()
p4 = vrep.simxGetObjectPosition(clientID, Dummy4_handle,-1, vrep.simx_opmode_blocking)
p4 = np.array(p4[1]).transpose()
#p5 = vrep.simxGetObjectPosition(clientID, Dummy5_handle,-1 , vrep.simx_opmode_blocking)
#p5 = np.array(p5[1]).transpose()

#print(p0)
#print(p1)
#print(p2)
#print(p3)
#print(p4)
#print(p5)
#p0 = vrep.simxGetObjectPosition(clientID, Dummy0_handle, -1, vrep.simx_opmode_blocking)
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Get the current value of the first joint variable
result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta1))

result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta2))

result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta3))

result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta4))

result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta5))

result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta6))



theta_goal1 = theta1 + (np.pi / 2)
theta_goal2 = theta2 + (np.pi / 2)
theta_goal3 = theta3 + (np.pi / 2)
theta_goal4 = theta4 + (np.pi / 2)
theta_goal5 = theta5 + (np.pi / 2)
theta_goal6 = theta6 + (np.pi / 2)





# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + (np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta2 + -(np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 + (np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4 + (3*np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta5 + (np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(2)

vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4 + (np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta6 + (np.pi), vrep.simx_opmode_oneshot)

result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 + (np.pi / 2), vrep.simx_opmode_oneshot)

result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)

time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + (np.pi), vrep.simx_opmode_oneshot)



r_1 = 0.5*0.2
p = np.zeros((3,5))
p[0:3,0] = p0
p[0:3,1] = p1
p[0:3,2] = p2
p[0:3,3] = p3
p[0:3,4] = p4
#p[0:3,5] = p5
#p = [p0,p1,p2,p3,p4,p5]
print(p)
# Wait two seconds

#print('made it!')
n= 0
for i in range(3):
    for j in range(i+2,5):
        if are_spheres_colliding(p[0:3,i], p[0:3,j], r_1, r_1):
            print('COLLIDING!')
            n=1
if(n==0):
    print('no collision')
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 - (np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
p0 = vrep.simxGetObjectPosition(clientID, Dummy0_handle, -1, vrep.simx_opmode_blocking)
p0 = np.array(p0[1]).transpose()
p1 = vrep.simxGetObjectPosition(clientID, Dummy1_handle, -1,vrep.simx_opmode_blocking)
p1 = np.array(p1[1]).transpose()
p2 = vrep.simxGetObjectPosition(clientID, Dummy2_handle,-1,vrep.simx_opmode_blocking)
p2 = np.array(p2[1]).transpose()
p3 = vrep.simxGetObjectPosition(clientID, Dummy3_handle,-1, vrep.simx_opmode_blocking)
p3 = np.array(p3[1]).transpose()
p4 = vrep.simxGetObjectPosition(clientID, Dummy4_handle,-1, vrep.simx_opmode_blocking)
p4 = np.array(p4[1]).transpose()
#p5 = vrep.simxGetObjectPosition(clientID, Dummy5_handle,-1 , vrep.simx_opmode_blocking)
#p5 = np.array(p5[1]).transpose()
r_1 = 0.5*0.2
p = np.zeros((3,5))
p[0:3,0] = p0
p[0:3,1] = p1
p[0:3,2] = p2
p[0:3,3] = p3
p[0:3,4] = p4
#p[0:3,5] = p5
#p = [p0,p1,p2,p3,p4,p5]
print(p)
# Wait two seconds

#print('made it!')
n=0
for i in range(3):
    for j in range(i+2,5):
        if are_spheres_colliding(p[0:3,i], p[0:3,j], r_1, r_1):
            print('COLLIDING!')
            n=1
if(n==0):
    print('no collision')
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)



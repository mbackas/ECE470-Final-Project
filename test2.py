import forwardKinematics
import vrep
import time
import numpy as np

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

M = forwardKinematics.forwardKinematicsUr3(np.array([theta_goal1,theta_goal2,theta_goal3,theta_goal4,theta_goal5,theta_goal6]))
objHand, result = vrep.simxGetObjectHandle(clientID,'frame',vrep.simx_opmode_blocking)
result = vrep.simxSetObjectPosition(clientID,objHand,-1,(M[0,3],M[1,3],M[2,3]),vrep.simx_opmode_blocking)



# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + (np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta2 + -(np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 + (np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4 + (np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta5 + (np.pi / 2), vrep.simx_opmode_oneshot)
time.sleep(2)
# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta6 + (np.pi / 2), vrep.simx_opmode_oneshot)

result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 - (np.pi / 2), vrep.simx_opmode_oneshot)

result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)

time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + (np.pi), vrep.simx_opmode_oneshot)





# Wait two seconds
time.sleep(5)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)

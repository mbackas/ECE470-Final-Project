clear all;
vrep = remApi('remoteApi');

vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

% Connect to V-REP (raise exception on failure)


%returnCode= vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
project(vrep, clientID);
# ECE470-Final-Project
We simulate 2 UR10 robots to throw and retrieve a ball back and forth

Here are the steps needed to run our simulation in V-REP
1. Clone or download this repository
1. download [V-REP](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_4_0_Mac.zip)
2. extract the file and rename vrep, for brevity.
3. move my_ur10.ttt to vrep->scenes
3. in the vrep directory, run V-REP from the terminal command line with `./vrep.app/Contents/MacOS/vrep`
4. Click File-> Open Scene-> my_ur10
5. If python 3 is not istalled, [download and install it](https://www.anaconda.com)
5. From the vrep_code directory, run `python test.py`




## Simulation 2: forward kinematics
0. complete above steps
1. pull the repository
999. copy the new files forwardKinematics.py and test2.py into vrep_code
3. run `python test2.py`

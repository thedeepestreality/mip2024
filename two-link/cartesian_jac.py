import pybullet as p
import numpy as np
import time

dt = 1/240 # pybullet simulation step
# q0 = np.pi - np.deg2rad(5)   # starting position (radian)
# q0 = np.deg2rad(15)   # starting position (radian)
th0 = 0.5
jIdx = [1,3]
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = th0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0
u = 0
xd = 0.1
zd = 1.0
L = 0.5

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./two-link.urdf", useFixedBase=True)

# turn off internal damping
numJoints = p.getNumJoints(boxId)
for i in range(numJoints):
    print(f"{i} {p.getJointInfo(boxId, i)[1]} {p.getJointInfo(boxId, i)[12]}")

# go to the starting position
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetPositions=[th0,th0], controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetVelocities=[0,0], controlMode=p.VELOCITY_CONTROL, forces=[0,0])
for t in logTime[1:]:
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, controlMode=p.TORQUE_CONTROL, force=f)
    jointState = p.getJointStates(boxId, jIdx)
    th1 =  jointState[0][0]
    th2 =  jointState[1][0]
    logCtrl[idx] = u

    linkState = p.getLinkState(boxId, linkIndex=4)
    xSim2 = linkState[0][0]
    zSim2 = linkState[0][2]
    print(f"SIMUL2: {xSim2} {zSim2}")

    J = np.array([[-L*np.cos(th1)-L*np.cos(th1+th2), -L*np.cos(th1+th2)], 
                  [L*np.sin(th1)+L*np.sin(th1+th2), L*np.sin(th1+th2)]])
    
    # w = 10*np.linalg.inv(J) @ -np.array([[xSim2-xd],[zSim2-zd]])
    w = 10*np.linalg.inv(J) @ np.array([[0.1],[0]])
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetVelocities=[w[0,0],w[1,0]], controlMode=p.VELOCITY_CONTROL)
    
    p.stepSimulation()

    jointState = p.getJointStates(boxId, jIdx)
    th1 = jointState[0][0]
    dth1 = jointState[0][1]
    logVel[idx] = dth1
    idx += 1
    logPos[idx] = xSim2
    time.sleep(dt)

logVel[idx] = p.getJointStates(boxId, jIdx)[0][1]
logCtrl[idx] = u

import matplotlib.pyplot as plt

plt.subplot(3,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
#plt.plot([logTime[0],logTime[-1]],[thd,thd],'r', label='refPos')
plt.legend()

plt.subplot(3,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.subplot(3,1,3)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simCtrl")
plt.legend()

plt.show()

p.disconnect()

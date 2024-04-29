import pybullet as p
import numpy as np

dt = 1/240 # pybullet simulation step
th0 = 0.1
thd = 0.5
jIdx = 1
kp = 20
kd = 10
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = th0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0
tau = 0

L = 0.5
g = 10
m = 1
k = 0.5

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./pendulum.urdf", useFixedBase=True)

# turn off internal damping
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for t in logTime[1:]:
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, controlMode=p.TORQUE_CONTROL, force=f)
    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    e = th1-thd
    u = -kp*e - kd*dth1
    tau = m*g*L*np.sin(th1) + k*dth1 + m*L*L*(u)

    logCtrl[idx] = tau

    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx,
        controlMode=p.TORQUE_CONTROL, 
        force=tau
    )
    
    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = dth1
    idx += 1
    logPos[idx] = th1

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logCtrl[idx] = tau

import matplotlib.pyplot as plt

plt.subplot(3,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0],logTime[-1]],[thd,thd],'r', label='refPos')
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

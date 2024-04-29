import pybullet as p
import numpy as np

dt = 1/240 # pybullet simulation step
th0 = np.deg2rad(15)
thd = np.deg2rad(30)
T = 2
jIdx = 1
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = np.rad2deg(th0)
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0
u = 0

# print(f"place: {K}")
# print(f"place: {K@np.array([0,0])}")

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
dth1_prev = 0
a2 = 3/T**2
a3 = -2/T**3
for t in logTime[1:]:
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, controlMode=p.TORQUE_CONTROL, force=f)
    jointState = p.getJointState(boxId, jIdx)
    #s = t/T
    s = a2*t**2 + a3*t**3
    u = th0 + s*(thd-th0)
    
    if t > T:
        u=thd
    dth1 = jointState[1]
    acc = (dth1 - dth1_prev)/dt
    dth1_prev = dth1
    logCtrl[idx] = np.rad2deg(acc)
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx, 
        controlMode=p.POSITION_CONTROL, 
        targetPosition=u
    )

    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = np.rad2deg(dth1)
    idx += 1
    logPos[idx] = np.rad2deg(th1)

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logCtrl[idx] = np.rad2deg(acc)

import matplotlib.pyplot as plt

plt.subplot(3,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0],logTime[-1]],[np.rad2deg(thd),np.rad2deg(thd)],'r', label='refPos')
plt.legend()

plt.subplot(3,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.subplot(3,1,3)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simAcc")
plt.legend()

plt.show()

p.disconnect()

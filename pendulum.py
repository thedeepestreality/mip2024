import pybullet as p
import time

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
jIdx = 1
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./pendulum.urdf", useFixedBase=True)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
while True:
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()

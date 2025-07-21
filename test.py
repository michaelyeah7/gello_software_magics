# import franky
from franky import *
import franky

robot = Robot("172.16.0.2")
robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(velocity=0.15, acceleration=0.1, jerk=0.03)
quat = [1.0,0.0,0.0,0.0]
# motion =  CartesianMotion(Affine([0.50,-0.2,0.18],quat))
joint_motion = JointMotion([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7])

robot.move(joint_motion)

robot.close()
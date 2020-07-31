#!/usr/bin/env python2

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""universal_robot_ros controller."""

import argparse
import rospy
import sys

from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from kinect_helper import KinectHelper
from robotiq_gripper_3f import Robotiq3F
from rosgraph_msgs.msg import Clock


parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node(arguments.nodeName, disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Robot()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
# These joints must match with the list in ros_controllers.yaml!
armJointNames = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
armTrajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, armJointNames, 'arm_controller')
armTrajectoryFollower.start()
# These joints must match with the list in ros_controllers.yaml!
handJointNames = [
    'finger_1_joint_1',
    'finger_2_joint_1',
    'finger_middle_joint_1',
    'palm_finger_1_joint',
    'palm_finger_2_joint'
]
handTrajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, handJointNames, 'hand_controller')
handTrajectoryFollower.start()

# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = 64 #int(robot.getBasicTimeStep())

# Kinect camera
kinect = KinectHelper(robot.getCamera('kinect color'),
    robot.getRangeFinder('kinect range'), timestep)

# Gripper
gripper = Robotiq3F(robot, timestep)

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    armTrajectoryFollower.update()
    handTrajectoryFollower.update()
    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    clockPublisher.publish(msg)
    # Publish camera
    kinect.broadcast_color_image()
    kinect.broadcast_depth_image()

sys.exit(0)
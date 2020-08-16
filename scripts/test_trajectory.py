#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import math


def run():

  # ROS initialize
  rospy.init_node('test_trajectory', anonymous=False)

  # 
  rate = rospy.Rate(10)
  cmd_pub = rospy.Publisher('/arm_controller/command', JointTrajectory,
                            queue_size=10)

  # Trajectory initialize
  traj = JointTrajectory()
  point = JointTrajectoryPoint()

  traj.joint_names = [
    'elbow_1_joint', 
    'shoulder_1_joint',
    'shoulder_2_joint',
    'wrist_1_joint', 
    'wrist_2_joint', 
    'wrist_3_joint'
    ]
  
  point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  point.time_from_start = rospy.Duration(0.1, 0.0)

  traj.points.append(point)

  diff = math.pi /10.0

  while not rospy.is_shutdown():

    point.positions[0] += diff

    if point.positions[0] > math.pi / 2.0:
      diff = - math.pi / 10.0
    elif point.positions[0] < - math.pi / 2.0:
      diff = math.pi /10.0

    print(traj.points[0].positions)

    cmd_pub.publish(traj)

    rate.sleep()


if __name__ == '__main__':

  try:
    run()
  except rospy.ROSInterruptException as ex:
    rospy.logerr(ex)
  except Exception as ex:
    rospy.logerr(ex)
  except:
    ropy.logerr('unknown error')
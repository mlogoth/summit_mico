#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def moveJoint (jointcmds,prefix,nbJoints):
  topic_name = '/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingers (jointcmds,prefix,nbJoints):
  topic_name = '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

if __name__ == '__main__':
  try:    
    rospy.init_node('move_mico_using_trajectory_msg')		
    rospy.sleep(1)

    #home robots
    moveJoint ([0.0,2.9,1.3,4.2,1.4,0.0],'mico',6)
    moveFingers ([1,1],'mico',2)
  except rospy.ROSInterruptException:
    print "program interrupted before completion"

#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64

def moveJoint (jointcmds,prefix,nbJoints):
  for i in range(1,7):
    topic_name = '/joint_'+str(i)+'_position_controller/command'
    print topic_name
    pub = rospy.Publisher(topic_name, Float64, queue_size=1)
    jointCmd = Float64()  
    jointCmd.data = jointcmds[i-1]
    rate = rospy.Rate(100)
    count = 0
    while (count < 50):
      rospy.loginfo(jointCmd)
      pub.publish(jointCmd)
      count = count + 1
      rate.sleep()


def moveFingers (jointcmds,prefix,nbJoints):
  for i in range(1,3):
    topic_name = '/finger_'+str(i)+'_position_controller/command'
    pub = rospy.Publisher(topic_name, Float64, queue_size=1)
    jointCmd = Float64()  
    jointCmd.data = jointcmds[i-1]
    rate = rospy.Rate(100)
    count = 0
    while (count < 50):
      pub.publish(jointCmd)
      count = count + 1
      rate.sleep()


if __name__ == '__main__':
  try:    
    rospy.init_node('move_mico_using_trajectory_msg')		
    rospy.sleep(2)

    #home robots
    moveJoint ([0.0,2.9,1.3,4.2,1.4,0.0],'mico',6)
    moveFingers ([1,1],'mico',2)
  except rospy.ROSInterruptException:
    print "program interrupted before completion"

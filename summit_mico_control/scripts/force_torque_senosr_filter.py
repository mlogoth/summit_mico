#! /usr/bin/env python
"""Re-publish force torque with filtering"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped


class WrenchFilterRepublish:
    def __init__(self,topic_to_subscribe,topic_to_publish,a=np.zeros(6)):
        
        self.sub = rospy.Subscriber(topic_to_subscribe,WrenchStamped,self.callback,queue_size=100)
        self.pub = rospy.Publisher(topic_to_publish,WrenchStamped,queue_size=100)
        self.wrench = np.zeros(6)
        self.a = a
    
    def callback(self,msg):
        # Forces        
        self.wrench[0] = (1.0-self.a[0])*msg.wrench.force.x + self.a[0]*self.wrench[0]
        self.wrench[1] = (1.0-self.a[1])*msg.wrench.force.y + self.a[1]*self.wrench[1]
        self.wrench[2] = (1.0-self.a[2])*msg.wrench.force.z + self.a[2]*self.wrench[2]
        # Torques
        self.wrench[3] = (1.0-self.a[3])*msg.wrench.torque.x + self.a[3]*self.wrench[3]
        self.wrench[4] = (1.0-self.a[4])*msg.wrench.torque.y + self.a[4]*self.wrench[4]
        self.wrench[5] = (1.0-self.a[5])*msg.wrench.torque.z + self.a[5]*self.wrench[5]
        # publish
        
        filtered = WrenchStamped()
        filtered.header = msg.header
        filtered.wrench.force.x = self.wrench[0]
        filtered.wrench.force.y = self.wrench[1]
        filtered.wrench.force.z = self.wrench[2]
        filtered.wrench.torque.x = self.wrench[3]
        filtered.wrench.torque.y = self.wrench[4]
        filtered.wrench.torque.z = self.wrench[5]
        self.pub.publish(filtered)        
        


if __name__ == '__main__':
        
    try:
        rospy.init_node('force_torque_filtering')
        rate = rospy.Rate(250)
        a = 0.5*np.ones(6);
        ft1 = WrenchFilterRepublish("/mm1_mico_joint_6/ft_sensor","/mm1_mico_joint_6/ft_sensor_filtered",a)
        while not rospy.is_shutdown() :
            rate.sleep()
        
        rospy.spin()
    except rospy.ROSInterruptException: pass
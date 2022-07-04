#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class MUX:
    def __init__(self):
        rospy.init_node('cmd_vel_mux')
        rospy.Subscriber('nav_cmd_vel', Twist, self.callback1)
        rospy.Subscriber('fire_cmd_vel', Twist, self.callback2)
        rospy.Subscriber('flag', Int8, self.flag_callback)
        
        self.rate = rospy.Rate(10)
        self.nav_twist = Twist()
        self.fire_twist = Twist()
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.priority = 0
    
    def callback1(self, msg):
        self.nav_twist.linear.x = msg.linear.x
        self.nav_twist.angular.z = msg.angular.z
    
    def callback2(self, msg):
        self.fire_twist.linear.x = msg.linear.x
        self.fire_twist.angular.z = msg.angular.z

    def flag_callback(self, msg):
        if msg.data == 0:
            self.priority = 0
        
        else:
            self.priority = 1
    
    def spin(self):
        if self.priority == 0:
            self.pub.publish(self.nav_twist)
        
        else:
            self.pub.publish(self.fire_twist)
        
        self.rate.sleep()
        rospy.spin()
        
if __name__ == '__main__':
    mux = MUX()
    mux.spin()

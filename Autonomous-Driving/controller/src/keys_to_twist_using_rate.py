#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w':[0.0, 1.0], 'x':[-0.0, -1.0], 'a':[0.8, 0.0], 'd':[-0.8, 0.0], 's':[0.0, 0.0]}

g_last_twist = Twist()

def keys_cb(msg, twist_pub):
    global g_last_twist
    if len(msg.data) == 0:
        return

    vels = key_mapping[msg.data[0]]
    g_last_twist.angular.z = vels[0]
    g_last_twist.linear.x = vels[1]
    twist_pub.publish(g_last_twist)

if __name__=="__main__":
    rospy.init_node("keys_to_twist")
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb, twist_pub)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()

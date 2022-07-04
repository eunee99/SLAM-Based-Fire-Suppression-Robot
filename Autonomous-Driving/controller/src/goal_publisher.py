#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

class map_navigation():
    def __init__(self):
        self.fire_location_x = 0.0
        self.fire_location_y = 0.0
        self.reached_flag = 0
        self.ros_init()

    def ros_init(self):
        rospy.init_node("fire_navigation", anonymous=False)
        self.pub = rospy.Publisher('flag', Int8, queue_size=10)
        self.rate = rospy.Rate(1)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def moveToGoal(self, x_goal, y_goal):
        rospy.loginfo(rospy.get_name() + 'started!')
        print('Fire Detected!! Go To Point Of Fire')
        
        while not self.ac.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo('Waiting for the move_base action server to come up')
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving toward the goal
        goal.target_pose.pose.position = Point(x_goal, y_goal, 0)  # find the coordinate
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        rospy.loginfo('Sending goal location ...')
        
        # sending destination info
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        
        # wait for result
        self.ac.wait_for_result()

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            self.reached_flag = 1
            self.pub.publish(1)
            rospy.loginfo('You have reached the destination')
        
        else:
            self.pub.publish(0)
            rospy.loginfo('The robot failed to reach the destination')

if __name__=='__main__':
    navi = map_navigation()
    navi.moveToGoal(navi.fire_location_x, navi.fire_location_y)
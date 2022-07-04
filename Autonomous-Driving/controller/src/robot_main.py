#!/usr/bin/env python

import rospy
from robot_sub import fire_detector
from fire_detection import fire_detection
from goal_publisher import map_navigation


# if robot_sub == 1 --> go to goal position
# if reached to goal position --> setting motor position
# if motor position complete --> launch fire extinguisher !!!!

if __name__ == '__main__':
    fd =  fire_detector()
    navi = map_navigation()
    
    fd.loop_forever()
    
    if fd.detection_flag == 1:
        navi.moveToGoal(navi.fire_location_x, navi.fire_location_y)

    if navi.reached_flag == 1:
            fire = fire_detection()
            rospy.spin()
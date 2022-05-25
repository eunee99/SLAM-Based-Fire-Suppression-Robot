#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry as lg
import math

rospy.init_node('laserscan_to_pointcloud')

lp = lg.LaserProjection()

pc_pub = rospy.Publisher('pcl', PointCloud2, queue_size = 10)

def scan_cb(msg):
	pc2_msg = lp.projectLaser(msg)
	pc_pub.publish(pc2_msg)

	point_generator = pc2.read_points(pc2_msg)

	total = 0.0
	num = 0
	for point in point_generator:
		if not math.isnan(point[2]):
			total += point[2]
			num += 1

	point_list = pc2.read_points_list(pc2_msg)


rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size = 1)
rospy.spin()

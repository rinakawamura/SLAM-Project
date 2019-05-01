#!/usr/bin/env python
import rospy
import random
import numpy as np

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point, Quaternion

ns = "map_segmentation"
id_no = random.randint(0, 2**31)

draw_object = None

def map_callback(data):
	rospy.loginfo("inside map callback")
	width = data.info.width
	height = data.info.height
	res = data.info.resolution

	m = np.array(data.data).reshape((height, width))
	rospy.loginfo(np.shape(m))
	
	obstacles = m >= 90
	rospy.loginfo(obstacles.sum())
	obs_loc = obstacles.nonzero()
	rospy.loginfo(np.shape(obs_loc))
	obs_count = np.shape(obs_loc)[1]

	nearby = []
	threshold = 0.05
	for i in range(obs_count):
		for j in range(i+1, obs_count):
			x1, y1 = obs_loc[0][i], obs_loc[1][i]
			x2, y2 = obs_loc[0][j], obs_loc[1][j]
			if (x1-x2)**2 + (y1-y2)**2 < threshold/(res**2):
				nearby.append((i, j))
	rospy.loginfo(len(nearby))

	draw = Marker()
	draw.header = data.header
	draw.ns = ns
	draw.id = id_no
	draw.type = Marker.LINE_LIST
	draw.pose = data.info.origin
	draw.scale = Vector3(0.03, 0, 0)
	draw.lifetime = rospy.Duration(5)
	draw.frame_locked = False
	points = []
	colors = []
	for pair in nearby:
		i, j = pair
		points.append(Point(res*obs_loc[1][i], res*obs_loc[0][i], 0))
		points.append(Point(res*obs_loc[1][j], res*obs_loc[0][j], 0))
		color = ColorRGBA(0, 0, 1, 0.7)
		colors.append(color)
		colors.append(color)
	draw.points = points
	draw.colors = colors

	global draw_object
	draw_object = draw
	rospy.loginfo("Finished map callback")
	

def listener():
	rospy.init_node('segmentation', anonymous=False)
	rospy.Subscriber('/map', OccupancyGrid, map_callback)
	walls = rospy.Publisher('/walls', Marker, queue_size=1)

	frequency = 3  # Hz
	rate = rospy.Rate(frequency)
	while not rospy.is_shutdown():
		#rospy.loginfo("inside loop")
		if draw_object is not None:
			walls.publish(draw_object)
			rospy.loginfo("Published data")

		rate.sleep()

if __name__ == '__main__':
    listener()

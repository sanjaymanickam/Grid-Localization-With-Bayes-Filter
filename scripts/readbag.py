#!/usr/bin/env python

import rosbag
import roslib
import rospy
import sys

if __name__ == "__main__":
	rospy.init_node('readbag')
	counter = {'Movements':1,'Observations':1}
	try:
		bag = rosbag.Bag('../bag/grid.bag')
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			print '#'*60
			print topic,'#'+str(counter[topic])
			print msg
			print t	
			counter[topic] += 1
	finally:
		bag.close()

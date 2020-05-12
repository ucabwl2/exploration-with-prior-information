#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg
    

if __name__ == '__main__':
	rospy.init_node('velodyne_tf_broadcaster')
	br = tf.TransformBroadcaster()
	#br_odom = tf.TransformBroadcaster()
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
		
		br.sendTransform((0, 0, 0),
		tf.transformations.quaternion_from_euler(0, 3.141, 0),
		rospy.get_rostime(),
		"velodyne",
		"base_link")
		
		#br.sendTransform((0, 0, 0),
		#tf.transformations.quaternion_from_euler(0, 0, 0),
		#rospy.Time.now(),
		#"base_link",
		#"odom")

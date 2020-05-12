#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import time

msg = """
Here to save gustav 
"""

if __name__=="__main__":
    	#settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('safe_gustav')

	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		while not rospy.is_shutdown():
			time.sleep(2)
			print "Publishing stop Twist"
			twist = Twist()
			twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			pub.publish(twist)

	except rospy.ROSInterruptException:
		pass

	finally:
		print "Closing down"
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		#termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



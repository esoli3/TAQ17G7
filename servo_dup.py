#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	if data.data == True
		rospy.loginfo(rospy.get_caller_id() + "Servo position horizontal")
	elif data.data == False	
		rospy.loginfo(rospy.get_caller_id() + "Servo position vertical")
	
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("servo_position", Bool, callback)
	rospy.spin()
	
if __name__ == '__main__':
	listener()
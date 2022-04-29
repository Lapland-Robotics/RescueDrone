#!/usr/bin/env python
import rospy
from humandetection.msg import Num

def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	rospy.loginfo("%d", data.Command)
 
def listener():
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("chatter", Num, callback)
# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

def talker(tmp):
	rospy.init_node('listener', anonymous=True)
	pub = rospy.Publisher('chatter', Num, queue_size=10)
	pub_val = Num()
	pub_val.ID = 4
	pub_val.Command = tmp
	rospy.loginfo(pub_val)
	pub.publish(pub_val)

if _name_ == '_main_':
	while True:
		val = input("Put in command: ")
		talker(val)
		listener()
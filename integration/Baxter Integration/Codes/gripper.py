#!/usr/bin/env python
import rospy
import baxter_interface

from std_msgs.msg import String

#class defined for gripper
class Helper(object):
       
	def __init__(self):

        	self._lgrip = baxter_interface.Gripper("left")
        	self._rgrip = baxter_interface.Gripper("right")
#        DEFINE THE VARIABKE FROM THE MESSAGE
        	self._command = "hsgx"

#	update the command value
	def callback(self, data):
	    	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		self._command = data.data
		return

	#check the value and take action
	def check(self):
	    
		if self._command == "Negative":
		        self._rgrip.open()
		elif self._command == "Positive":
		        self._rgrip.open()

	
#create an instance for gripper
def main():

	rospy.init_node("gripper")
        h = Helper()
 	rospy.Subscriber("chatter", String, h.callback)
        #h.callback(data)
       # Helper.callback(h, data)
	while not rospy.is_shutdown():
	      h.check()
	#rospy.spin()


if __name__ == "__main__":
	  #listener()
       main()
	   

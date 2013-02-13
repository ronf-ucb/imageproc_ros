# message test in debugger to checkout format
import sys
sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
import roslib
import rospy
import geometry_msgs.msg
import turtlesim.msg

msg = geometry_msgs.msg
m = geometry_msgs.msg.TransformStamped()
# rospy.init_node('Test_msg')
#time = rospy.Time.now()
time = 1000
vec = (1,0,0)
quat = (0,0,0,1)
print m
m.transform.translation.x = -3
m.transform.translation.y = -3
m.transform.translation.z = -3
print m


# message test in debugger to checkout format
import sys
sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
import roslib
import rospy
import rosgraph
import geometry_msgs.msg
import turtlesim.msg
import sensor_msgs.msg


msg = geometry_msgs.msg
m = geometry_msgs.msg.TransformStamped()
import rosgraph
rospy.init_node('Test_msg')
# time = rospy.Time.now()

time = 1000
vec = (1,0,0)
quat = (0,0,0,1)
# print m
m.transform.translation.x = -3
m.transform.translation.y = -3
m.transform.translation.z = -3
print m

smsg = sensor_msgs.msg.JointState()
smsg.position = [256,1024]  # motor pos
smsg.name = 'TurnerJoints'
smsg.velocity = [1023,235]   # back EMF
smsg.effort = [12, 375]  # PWM command

imsg = sensor_msgs.msg.Imu()
imsg.linear_acceleration.x = 230
imsg.linear_acceleration.y = 120
imsg.angular_velocity.x = 12.7




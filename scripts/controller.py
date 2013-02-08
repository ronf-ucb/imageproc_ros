# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#!/usr/bin/env python

# should get pose message from optitrack, and calculate commanded
# robot velocity to get to origin 

import roslib
roslib.load_manifest('Turner25')
import rospy

import tf
import geometry_msgs.msg
import turtlesim.msg
# need Pose and TransformStamped

def handle_turner_pose(msg, robotname):
    x_est = msg.x
    y_est = msg.y
    theta_est = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('Control')
# add default value
    robotname = 'VelociRoACH1'
    rospy.Subscriber('/optitrack/pose',
                     geometry_msgs.msg.TransformStamped,
                     handle_turner_pose,
                     robotname)
    pub= rospy.Publisher('velCMD', turtlesim.msg.Velocity)
    rospy.spin()

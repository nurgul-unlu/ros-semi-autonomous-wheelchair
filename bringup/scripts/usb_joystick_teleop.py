#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import tf

WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.175)  # m (örneğin 0.175)
COUNTS_PER_REV = rospy.get_param('~counts_per_rev', 600)
WHEEL_BASE = rospy.get_param('~wheel_base', 0.43)  # m

class WheelOdom:
    def __init__(self):
        rospy.init_node('wheel_odom_node')
        self.sub = rospy.Subscriber('/wheel_encoder_counts', Int32MultiArray, self.cb)
        self.pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=5)
        self.x = self.y = self.th = 0.0
        self.prev_l = None; self.prev_r = None
        self.prev_time = rospy.Time.now()
    def cb(self,msg):
        left = msg.data[0]; right = msg.data[1]
        now = rospy.Time.now()
        if self.prev_l is None:
            self.prev_l, self.prev_r = left, right; self.prev_time = now; return
        dt = (now - self.prev_time).to_sec()
        dl = (left - self.prev_l) / float(COUNTS_PER_REV) * 2*math.pi*WHEEL_RADIUS
        dr = (right - self.prev_r) / float(COUNTS_PER_REV) * 2*math.pi*WHEEL_RADIUS
        self.prev_l, self.prev_r = left, right
        self.prev_time = now
        dcenter = (dr + dl)/2.0
        dth = (dr - dl)/WHEEL_BASE
        self.x += dcenter * math.cos(self.th + dth/2.0)
        self.y += dcenter * math.sin(self.th + dth/2.0)
        self.th += dth
        odom = Odometry()
        odom.header.stamp = now; odom.header.frame_id='odom'; odom.child_frame_id='base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.th)
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y=q[1]; odom.pose.pose.orientation.z=q[2]; odom.pose.pose.orientation.w=q[3]
        odom.twist.twist.linear.x = dcenter/dt if dt>0 else 0.0
        odom.twist.twist.angular.z = dth/dt if dt>0 else 0.0
        self.pub.publish(odom)

if __name__=='__main__':
    WheelOdom(); rospy.spin()

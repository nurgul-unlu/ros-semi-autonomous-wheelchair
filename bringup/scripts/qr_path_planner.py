#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

# --- QR noktaları (map frame'inde) ---
QR_POINTS = {
    "QR5": (3.224, -0.824, 0.0),
    "QR2": (1.7564420700073242, -1.1592153310775757, 0.0),
    "QR3": (0.01982712745666504, -1.3603174686431885, 0.0)
}

class QRPlannerSimple:
    def __init__(self):
        rospy.init_node("qr_path_planner")

        # RViz 2D Nav Goal ile aynı topic
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )

        # Robot hareket etmesin diye
        self.stop_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1
        )

        # Hangi QR'a gideceğini buradan seçeceğiz
        rospy.Subscriber("/select_qr", String, self.on_qr_select)

        rospy.loginfo("QR Path Planner çalışıyor. Başlangıç QR1.")
        rospy.sleep(1.0)          # publisher hazır olsun
        self.send_goal("QR1")     # açılışta QR1'e hedef gönder

        # Robotu sürekli durduralım (sadece çizgi çıksın)
        rospy.Timer(rospy.Duration(0.1), self.stop_robot)

    def stop_robot(self, event):
        self.stop_pub.publish(Twist())

    def on_qr_select(self, msg):
        name = msg.data.strip()
        rospy.loginfo("Seçilen QR: %s" % name)
        self.send_goal(name)

    def send_goal(self, name):
        if name not in QR_POINTS:
            rospy.logwarn("Bilinmeyen QR ismi: %s" % name)
            return

        x, y, yaw = QR_POINTS[name]
        rospy.loginfo("%s hedefine goal gönderiliyor..." % name)

        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time.now()

        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        self.goal_pub.publish(ps)

if __name__ == "__main__":
    QRPlannerSimple()
    rospy.spin()


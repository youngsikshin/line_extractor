import rospy, math, numpy, tf
from nav_msgs.msg import Odometry

def callback(msg):
    q = msg.pose.pose.orientation
    transformBroadcaster.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0), (q.x, q.y, q.z, q.w), msg.header.stamp, "base_footprint", "odom")

if __name__ == '__main__':
    rospy.init_node("odom_to_tf")

    global transformBroadcaster
    transformBroadcaster = tf.TransformBroadcaster()

    subscriber = rospy.Subscriber("/odom", Odometry, callback)

    rospy.spin()
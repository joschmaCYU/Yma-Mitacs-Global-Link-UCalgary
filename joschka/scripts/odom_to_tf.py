#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


def handle_robot_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # On récupère l'heure exacte de la simulation
    t.header.stamp = msg.header.stamp

    # Le parent est le monde global
    t.header.frame_id = "world"

    # L'enfant est la racine de ton robot
    t.child_frame_id = "base_link"

    # On copie la position (x, y, z)
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    # On copie l'orientation (quaternion)
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("odom_to_tf_broadcaster")
    # On s'abonne au topic publié par le plugin Gazebo
    rospy.Subscriber("/odom", Odometry, handle_robot_pose)
    rospy.loginfo("odom to tf init")

    rospy.spin()

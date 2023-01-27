#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
tf_static_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1)

def tf_callback(msg:TFMessage):
    tf_msg = TFMessage()
    
    if msg.transforms[0].header.frame_id.startswith('dsr'):
        tf_msg = msg
    else:
        return
    
    for transform in tf_msg.transforms:
        transform.header.frame_id = transform.header.frame_id.split('/')[-1]
        transform.child_frame_id = transform.child_frame_id.split('/')[-1]

    tf_pub.publish(tf_msg)

def tf_static_callback(msg:TFMessage):
    tf_static_msg = msg
    tf_static_msg.transforms[0].header.frame_id = 'world'
    tf_static_msg.transforms[0].child_frame_id = 'base_0'
    tf_static_pub.publish(tf_static_msg)

if __name__ == "__main__":
    rospy.init_node('correct_rviz')
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    rospy.Subscriber('/tf_static', TFMessage, tf_static_callback)
    r = rospy.Rate(30)

    # tf_cam = TransformStamped()
    # tf_cam.header.frame_id = 'link6'
    # tf_cam.child_frame_id = 'camera_link'
    # tf_cam.transform.translation.x = 0.0
    # tf_cam.transform.translation.y = 0.0
    # tf_cam.transform.translation.z = 0.0
    # tf_cam.transform.rotation.x = 0.0
    # tf_cam.transform.rotation.y = 0.0
    # tf_cam.transform.rotation.z = 0.0
    # tf_cam.transform.rotation.w = 1.0

    while not rospy.is_shutdown():
        # tf_cam.header.stamp = rospy.Time.now()
        # tf_pub.publish(TFMessage([tf_cam]))

        r.sleep()
        rospy.spin()
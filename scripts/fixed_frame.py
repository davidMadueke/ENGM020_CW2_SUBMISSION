#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def set_fixed_frame():
    rospy.init_node('set_fixed_frame')
    rate = rospy.Rate(1)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # Create a TransformStamped message to set the fixed frame
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"  # Set your desired fixed frame here
        transform.child_frame_id = "base_link"  # Set your desired child frame here
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0  # Identity rotation

        # Publish the TransformStamped message as part of a TFMessage
        tf_message = TFMessage()
        tf_message.transforms.append(transform)

        # Publish the TFMessage to set the fixed frame
        tf_pub.publish(tf_message)

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
        set_fixed_frame()
    except rospy.ROSInterruptException:
        pass


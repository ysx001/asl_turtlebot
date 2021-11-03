#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D

def publisher():
    vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
    rospy.init_node('marker_node', anonymous=True)
    rate = rospy.Rate(1)
    
    marker = Marker()
    def goal_callback(data):
       marker.pose.position.x = data.x
       marker.pose.position.y = data.y
    
    rospy.Subscriber('/cmd_nav', Pose2D, goal_callback)
    while not rospy.is_shutdown():
        # marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker.id = 0

        marker.type = 2 # sphere

        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.8 # Don't forget to set the alpha!
        marker.color.r = 0.7
        marker.color.g = 0.2
        marker.color.b = 0.0
        
        vis_pub.publish(marker)
        print('Published marker!')
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Initialize ROS node
rospy.init_node('topics_quiz_node', anonymous=True)

# Publisher to cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def laser_callback(data):
    # Extract laser readings
    front_reading = data.ranges[len(data.ranges)//2]
    right_reading = data.ranges[0]
    left_reading = data.ranges[719]

    # Create Twist message
    cmd_vel_msg = Twist()

    # Implement logic based on laser readings
    if front_reading > 1.0:
        # No obstacle in front, move forward
        cmd_vel_msg.linear.x = 0.5
        cmd_vel_msg.angular.z = 0.0
    else:
        # Obstacle in front, adjust direction based on side readings
        cmd_vel_msg.linear.x = 0.0
        if right_reading < left_reading:
            # Obstacle at the right, turn left
            cmd_vel_msg.angular.z = 0.5
        else:
            # Obstacle at the left or center, turn right
            cmd_vel_msg.angular.z = -0.5

    # Publish the Twist message
    cmd_vel_pub.publish(cmd_vel_msg)

# Subscriber to laser scan topic
rospy.Subscriber('/kobuki/laser/scan', LaserScan, laser_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

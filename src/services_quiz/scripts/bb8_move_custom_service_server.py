#!/usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist
import time

def handle_bb8_square_movement(req):
    rospy.loginfo("Executing BB-8 square movement...")
    move_cmd = Twist()
   

    for _ in range(req.repetitions):
        for _ in range(4):  # Four sides for a square
            rospy.loginfo("Moving forward for {} seconds...".format(req.side))
            move_cmd.linear.x = -1
            bb8_velocity_publisher.publish(move_cmd)
            time.sleep(req.side*2)

            rospy.loginfo("Turning for 1 second...")
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0.81  # Adjust the angular velocity as needed
            bb8_velocity_publisher.publish(move_cmd)
            time.sleep(2.5)
            move_cmd.angular.z = 0  # Adjust the angular velocity as needed
            bb8_velocity_publisher.publish(move_cmd)
            time.sleep(2)

    rospy.loginfo("BB-8 square movement completed!")
    response = BB8CustomServiceMessageResponse()
    response.success = True
    return response.success

if __name__ == "__main__":
    rospy.init_node("bb8_square_movement_server")
    bb8_velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    service = rospy.Service("/move_bb8_in_square_custom", BB8CustomServiceMessage, handle_bb8_square_movement)
    rospy.loginfo("Ready to move BB-8 in a square.")
    rospy.spin()

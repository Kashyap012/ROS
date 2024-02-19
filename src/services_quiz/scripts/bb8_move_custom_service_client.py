#!/usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest







# rospy.init_node(bb8_square_movement_client)
# rospy.wait_for_service('/move_bb8_in_square_custom')
# bb8_square_movement_client = rospy.ServiceProxy("/move_bb8_in_square_custom", BB8CustomServiceMessage)
# bb8_square_movement_object = BB8CustomServiceMessageRequest()

# bb8_square_movement_object.side = 

def bb8_square_movement_client(side, repetitions):
    rospy.wait_for_service("/move_bb8_in_square_custom")
    try:
        bb8_square_movement = rospy.ServiceProxy("/move_bb8_in_square_custom", BB8CustomServiceMessage)
        response = bb8_square_movement(side, repetitions)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False

if __name__ == "__main__":
    rospy.init_node("bb8_square_movement_client")
    success_small_square = bb8_square_movement_client(side=1.0, repetitions=2)
    success_big_square = bb8_square_movement_client(side=2.0, repetitions=1)

    if success_small_square and success_big_square:
        rospy.loginfo("Square movements completed successfully.")
    else:
        rospy.logerr("Square movements failed.")


#!/usr/bin/env python

import rospy
import actionlib
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgFeedback
from std_msgs.msg import Empty

class CustomActionServer(object):

    def __init__(self):
        self.server = actionlib.SimpleActionServer('/action_custom_msg_as', CustomActionMsgAction, self.goal_callback, False)
        self.server.start()
        self.rate = rospy.Rate(1)

        # Initialize publishers
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)

    def goal_callback(self, goal):
        rospy.loginfo("Received goal: %s", goal.goal)

        if goal.goal == "TAKEOFF":
            self.takeoff()
        elif goal.goal == "LAND":
            self.land()
        else:
            rospy.logwarn("Invalid goal received: %s", goal.goal)
            self.server.set_aborted()

    def takeoff(self):
        rospy.loginfo("Drone is taking off...")
        self._pub_takeoff.publish(Empty())
        feedback = CustomActionMsgFeedback()
        feedback.feedback = "Taking off"
        self.server.publish_feedback(feedback)
        self.wait_for_land()

    def land(self):
        rospy.loginfo("Drone is landing...")
        self._pub_land.publish(Empty())
        feedback = CustomActionMsgFeedback()
        feedback.feedback = "Landing"
        self.server.publish_feedback(feedback)
        self.server.set_succeeded()

    def wait_for_land(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo("Preempt requested. Aborting takeoff.")
                self.server.set_preempted()
                return

            # Check if a land goal is received
            if self.server.current_goal.goal == "LAND":
                rospy.loginfo("Received land goal during takeoff. Transitioning to land.")
                self.land()
                return

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('action_server')
    server = CustomActionServer()
    rospy.spin()










# #! /usr/bin/env python

# import rospy
# import actionlib
# import time
# from actions_quiz.msg import CustomActionMsgAction , CustomActionMsgFeedback
# from std_msgs.msg import Empty


# class CustomActionServer(object):
   
#     _feedback = CustomActionMsgFeedback()


#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('/action_custom_msg_as', CustomActionMsgAction, self.goal_callback, False)
#         self.server.start()
#         self.rate = rospy.Rate(1)


#     def goal_callback(self,goal):


#         r = rospy.Rate(1)
#         success = True


#         self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
#         self._takeoff_msg = Empty()
#         self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
#         self._land_msg = Empty()


#         while goal.goal == "TAKEOFF":
#             rospy.loginfo("Drone is taking off...")
#             self._pub_takeoff.publish(self._takeoff_msg)
#             self._feedback.feedback = "taking off"
#             self.server.publish_feedback(self._feedback)


#             if goal.goal == "LAND":
#                 rospy.loginfo("Drone is landing...")
#                 self._pub_land.publish(self._land_msg)
#                 self._feedback.feedback = "landing"
#                 self.server.publish_feedback(self._feedback)
#                 break




# if __name__ == '__main__':
#     rospy.init_node('action_server')
#     server = CustomActionServer()
#     rospy.spin()

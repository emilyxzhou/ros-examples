#!/usr/bin/env python

import actionlib
import rospy

from actions_examples.msg import RollDieAction, RollDieGoal


class RollDieClient:

    _ACTION_NAME = "/examples/roll_die"

    def __init__(self, nums_wanted):
        self._nums_wanted = nums_wanted
        self._client = actionlib.SimpleActionClient(self._ACTION_NAME, RollDieAction)
        self._goal = RollDieGoal()

    def send_goal(self, num_rolls):
        self._client.wait_for_server()
        self._goal.num_rolls = num_rolls
        self._client.send_goal(self._goal, done_cb=self._done_cb, feedback_cb=self._feedback_cb)

        rospy.loginfo("Goal sent")

    def _done_cb(self, status, result):
        print("Action completed")
        print("Status: {}\nResult: {}".format(status, result.all_outcomes))

    def _feedback_cb(self, feedback):
        if int(feedback.last_outcome) in self._nums_wanted:
            self._client.cancel_goal()
            print("Received {}, goal canceled".format(feedback.last_outcome))


if __name__ == "__main__":
    rospy.init_node("ask_client")
    client = RollDieClient([])
    client.send_goal(5)
    rospy.spin()



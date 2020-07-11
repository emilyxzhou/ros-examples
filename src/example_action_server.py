#!/usr/bin/env python

import actionlib
import random
import rospy

from actions_examples.msg import RollDieAction, RollDieFeedback, RollDieResult


class DieRoller:

    _ACTION_NAME = "/examples/roll_die"

    def __init__(self):
        self._action_server = actionlib.SimpleActionServer(
            self._ACTION_NAME,
            RollDieAction,
            execute_cb=self._roll_die,
            auto_start=False)
        self._feedback = RollDieFeedback()
        self._result = RollDieResult()
        self._action_server.start()
        rospy.loginfo("Roll die server started")

    def _roll_die(self, goal):
        r = rospy.Rate(2)
        num_rolls = goal.num_rolls
        all_outcomes = []
        succeeded = True

        rospy.loginfo("Goal received: rolling die {} times".format(num_rolls))

        for _ in range(num_rolls):

            if self._action_server.is_preempt_requested():
                rospy.loginfo("Goal canceled")
                self._action_server.set_preempted()
                succeeded = False
                break

            current_outcome = random.randint(1, 6)
            all_outcomes.append(int(current_outcome))
            self._feedback.last_outcome = current_outcome
            self._action_server.publish_feedback(self._feedback)
            rospy.loginfo("Rolled a {}".format(current_outcome))
            r.sleep()

        if succeeded:
            self._result.all_outcomes = all_outcomes
            rospy.loginfo("{}: Succeeded".format(self._ACTION_NAME))
            self._action_server.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("ask_server")
    DieRoller()
    rospy.spin()

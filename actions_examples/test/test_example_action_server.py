#!/usr/bin/env python

import actionlib
import rospy
import rostest
import unittest

from actions_examples.msg import RollDieAction, RollDieGoal

PKG = "actions_examples"
NAME = "test_example_action_server"
_ROLL_DIE_ACTION = "/examples/roll_die"


class TestExampleActionServer(unittest.TestCase):

    def test_start_interaction_action(self):
        # set up action client
        client = actionlib.SimpleActionClient(
            _ROLL_DIE_ACTION,
            RollDieAction
        )

        # create goal
        goal = RollDieGoal()
        goal.num_rolls = 5

        # assert that client has conencted to server within 5 seconds
        self.assertTrue(
            client.wait_for_server(rospy.Duration(5))
            , "Could not connect"
        )

        client.send_goal(goal)

        # assert that goal has completed & result returned to client
        self.assertTrue(
            client.wait_for_result(rospy.Duration(5)),
            "Goal didn't finish"
        )
        self.assertEqual(actionlib.GoalStatus.SUCCEEDED, client.get_state())
        self.assertEqual(len(client.get_result().all_outcomes), 5)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestExampleActionServer)

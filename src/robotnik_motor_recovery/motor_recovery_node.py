#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from motor_recovery import MotorRecovery


def main():

    rospy.init_node("motor_recovery_node")

    rc_node = MotorRecovery()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()

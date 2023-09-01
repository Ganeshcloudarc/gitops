#!/usr/bin/env python3
import rospy
from autonav_core import AutoNav
import logging

def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])


if __name__ == "__main__":
    rospy.init_node("autonav_node")
    set_rospy_log_lvl(rospy.DEBUG)
    auto_nav = AutoNav()
    rospy.spin()


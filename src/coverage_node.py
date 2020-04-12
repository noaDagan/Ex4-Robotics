#!/usr/bin/python
#
# bug_node.py
#
#
import rospy, sys
from coverage import Coverage

if __name__ == "__main__":
    rospy.init_node("wander_bot_node", argv=sys.argv)
    forward_speed = 0.2
    rotation_speed = 0.5
    robot_size= 0.35
    print "coverageeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee nodeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeEE"
    # Taking params from launch file
    if rospy.has_param('~forward_speed'):
        forward_speed = rospy.get_param('~forward_speed')
    if rospy.has_param('~rotation_speed'):
        rotation_speed = rospy.get_param('~rotation_speed')
    if rospy.has_param('robot_size'):
        robot_size = rospy.get_param('~robot_size')
    my_coverage = Coverage()
    my_coverage.start()

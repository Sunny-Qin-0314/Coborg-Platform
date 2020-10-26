#!/usr/bin/env python

import rospy
from hebiros.srv import *

if "/hebiros/right_arm/feedback" not in rospy.get_published_topics():
    rospy.init_node('add_hebi_group', anonymous=True)
    group_client = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)

    group_srv = AddGroupFromNamesSrvRequest()
    group_srv.group_name = "right_arm"
    group_srv.names = ['base_temp', 'X-00476', 'elbow_temp']
    group_srv.families = ["*"]
    while not group_client.call(group_srv):
        continue

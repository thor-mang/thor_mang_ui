#!/usr/bin/env python

import rospy

from robotis_controller_msgs.msg import SyncWriteItem

rospy.init_node("test")

torque_pub = rospy.Publisher("/johnny5/robotis/sync_write_item", SyncWriteItem, queue_size=1000)

while torque_pub.get_num_connections() == 0:
    rospy.sleep(1)

joints = ['l_f0_j0', 'l_f1_j0', 'l_arm_wr_p', 'l_arm_wr_y', 'l_arm_wr_r', 'l_arm_el_y']
joints2 = ['head_lidar_spinning_joint']
value = 0

msg = SyncWriteItem()
msg.item_name = "torque_enable"
for joint in joints:
    msg.joint_name.append(joint)
    msg.value.append(value)
torque_pub.publish(msg)

# msg = SyncWriteItem()
# msg.item_name = "torque_enable"
# for joint in joints2:
#     msg.joint_name.append(joint)
#     msg.value.append(value)
# torque_pub.publish(msg)
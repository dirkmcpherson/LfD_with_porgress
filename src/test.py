from armpy import kortex_arm
import time
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("test")
arm = kortex_arm.Arm()
# print(arm.get_eef_pose())
joint_angles = rospy.wait_for_message("/my_gen3_lite/joint_states", JointState)
temp = JointState()
print(temp)
from armpy import kortex_arm
import time
import rospy
from sensor_msgs.msg import JointState
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from BC import rollout_trained_model
import time 



rospy.init_node("test")
arm = kortex_arm.Arm()
# print(arm.get_eef_pose())
bc_agent = rollout_trained_model.roll_out_BCND()
bc_agent.load_model("/home/hang/catkin_ws/src/ldf_with_progress/BC/")
#/home/hang/catkin_ws/src/ldf_with_progress/BC/trained models.pth 
arm.home_arm()
state = arm.get_joint_angles()
state = list(state)
print(state)

while True:
    print(arm.get_eef_pose())
    time.sleep(0.5)
    # action = bc_agent.get_action(state)
   
    # action = action.tolist()[0]
    # print("action:",action)
    # print(type(action), type(state))
    # for i in range(6):
    #     state[i] += action[i]/360*2*3.1415926
    # # print("state:",state)
    # arm.goto_joint_pose(state)
    # state = arm.get_joint_angles()
    # state = list(state)


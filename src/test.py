from armpy import kortex_arm
import time
import rospy
from sensor_msgs.msg import JointState
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from BC import rollout_trained_model
import time 

cup_pos = [[0.5516026616096497,0.3308942914009094,0.1391326606273651],[0.41698557138442993,0.34413501620292664,0.1396018534898758],
           [0.29829156398773193,0.35091647505760193,0.1672777384519577],[0.17075064778327942,0.36124327778816223,0.15736795961856842]]

icecream_pos  = [0.34541743993759155,-0.13331931829452515,0.08949728310108185]

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


arm.home_arm()
arm.open_gripper()
while True:
    # print(arm.get_eef_pose())
    # time.sleep(0.5)
    print("state:",state)
    state = state +[arm.get_gripper_position()]+ cup_pos[0]+icecream_pos
    action = bc_agent.get_action(state) 
    
    action = action.tolist()[0]
    action[6] = min(max(action[6],0),1)
    print("action:",action)
    print(type(action), type(state))
    for i in range(6):
        state[i] += action[i]/360*2*3.1415926
        # state[i] += action[i]
    # print("state:",state)
    arm.goto_joint_pose(state[:6])
    arm.send_gripper_command(mode = "position", value = action[6])
    state = arm.get_joint_angles()
    state = list(state)

    
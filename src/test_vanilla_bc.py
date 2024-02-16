from armpy import kortex_arm
import time
import rospy
import os
import sys
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from BC_vanilla.vanilla_bc import rollout_vanilla_BC

from BC import rollout_trained_model
from BC.utils import bc_utils
cup_pos = [[0.5516026616096497,0.3308942914009094,0.1391326606273651],[0.41698557138442993,0.34413501620292664,0.1396018534898758],
           [0.29829156398773193,0.35091647505760193,0.1672777384519577],[0.17075064778327942,0.36124327778816223,0.15736795961856842]]

icecream_pos  = [0.34541743993759155,-0.13331931829452515,0.08949728310108185]

rospy.init_node("test")
arm = kortex_arm.Arm()

# print(arm.get_eef_pose())
bc_agent = rollout_vanilla_BC()
bc_agent.load_model()
#/home/hang/catkin_ws/src/ldf_with_progress/BC/trained models.pth 
# arm.home_arm()

# print(state)
arm.home_arm()
arm.open_gripper()



# state = [pos.x, pos.y, pos.z]
# state = list(state) 
# quaternion = [arm.get_eef_pose().pose.orientation.x, arm.get_eef_pose().pose.orientation.y, arm.get_eef_pose().pose.orientation.z, arm.get_eef_pose().pose.orientation.w]
# time.sleep(1)
# state = list(state)
# print(state)
# txt_file_list = bc_utils.find_all_txt_files_with_eef()
# # print(txt_file_list)
# print(len(txt_file_list))
# message_list = bc_utils.read_txt_to_meaasages([txt_file_list[0]])
# print(len(message_list))
print(arm.get_gripper_position())
# print(rospy.wait_for_message("/my_gen3_lite/joint_states", JointState))

traj = []
cnt = 0
while True:
    # print(arm.get_eef_pose())
    # time.sleep(0.5)
    #time.sleep(0.2)
    #traj.append(state )
    # state = list(state) + cup_pos[0] +icecream_pos 
    # print("state:",state)
    state = arm.get_joint_angles()
    state = list(state) + [arm.get_gripper_position()]
    # traj.append(state[:3]+quaternion)
    print("state:",state)
    action = bc_agent.get_action(state) 
    
    action = action.tolist()[0]
    arm.goto_joint_pose(action[:6])

    # for i in range(3):
    #     state[i] += action[i]\
    # difference = [(action[i]-state[i]) for i in range(3)]
    # new_state = [state[i]+difference[i] for i in range(3)]
    # if new_state[2] < 0.12:
    #     new_state[2] = 0.12
    # if new_state[0] > 0.58:
    #     new_state[0] = 0.58
    # state = new_state
    # print("action:",action)
    # print(type(action), type(state))
    # state = action
    # for i in range(6):
    #     state[i] += action[i]/360*2*3.1415926
    #     state[i] += action[i]
    # print("state:",state)
    # arm.goto_joint_pose(state[:6])
    # arm.goto_eef_pose(action)
    if action[6] < 0.5:
        arm.close_gripper()
    else:
        arm.open_gripper()
    #arm.send_gripper_command(mode = "position", value = action[6])
    # pos = arm.get_eef_pose().pose.position
    # state = [pos.x, pos.y, pos.z]
    cnt += 1
    if cnt > 400:
        break
print(traj)
arm.goto_eef_waypoints(traj)
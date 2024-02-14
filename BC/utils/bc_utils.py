import rospy
import rosbag
from sensor_msgs.msg import JointState
import os
import torch
import csv
import random
import numpy as np
cup_pos = [[0.5516026616096497,0.3308942914009094,0.1391326606273651],[0.41698557138442993,0.34413501620292664,0.1396018534898758],
           [0.29829156398773193,0.35091647505760193,0.1672777384519577],[0.17075064778327942,0.36124327778816223,0.15736795961856842]]

icecream_pos  = [0.34541743993759155,-0.13331931829452515,0.08949728310108185]

def read_bag(bagdir):
    bag = rosbag.Bag(bagdir,'r')
    messages = []
    for _,msg,_ in bag.read_messages(topics=['/my_gen3_lite/joint_states']):
        temp = JointState()
        temp.header = msg.header
        temp.position = msg.position
        temp.velocity = msg.velocity
        temp.name = msg.name
        temp.effort = msg.effort
        messages.append(temp)
    return messages

def get_all_bag_files(file_path:str=None):
    if file_path is not None:
        file_dir = file_path
    else:
        # use default bag file location
        bag_file_list = []
        file_dir = os.path.dirname(
            os.path.dirname(
                os.path.dirname(
                    os.path.abspath(__file__)
                )))
        file_dir = os.path.join(file_dir,"bags/")
    # look through directory to find all bag files
    for root, dirs, files in os.walk(file_dir):
        for file in files:
            if file.endswith(".bag"):
                bag_file_list.append(os.path.join(root,file))
    return bag_file_list

def find_all_txt_files_with_eef(file_path:str=None):
    if file_path is None:
        # use default txt file locationtxt
        path = os.path.dirname(
            os.path.dirname(
                os.path.dirname(
                    os.path.abspath(__file__)
                )))
        file_dir = os.path.join(path,"bags/")
    else:
        path = file_path
    pos_txt_list = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith("poses.txt"):
                pos_txt_list.append(os.path.join(root,file))
    return pos_txt_list


def read_txt_to_meaasages(file_name_list):
    message_list = []
    for i in range(len(file_name_list)):
        data = np.genfromtxt(file_name_list[i],delimiter=' ')
        print(data.shape)
        message_list.append(data)
    return message_list


# read a list of bags, then convert them into a list of messages
def whole_bag_to_messages(bag_file_list):
    messages_list= []
    for i in range(len(bag_file_list)):
        messages = read_bag(bag_file_list[i])
        messages_list.append(messages)
    return messages_list

# read k trajectories to buffers, if available trajectory is less than k, read them repeatly
# buffer mus be k length
def read_one_trajectory_to_each_buffer(k ,buffers, message_list):
    top = 0
    for i in range(k):
        # print("i:{}".format(i))
        trajectory = message_list[top]
        buffer = buffers[i]
        for j in range(len(trajectory)):
            obs = list(trajectory[j].position[:7])
            action = list(trajectory[j].effort[:6])
            buffer.add_sample(obs,action)
        top = (top + 1) % len(message_list)

def spilt_traj_with_cup_pos_and_read_to_buffer(buffers,message_list,cup_idx_list,cup_idx):
    top = 0
    waypoints = []
    action_list = []
    for i in range(len(message_list)):
        prev_joint_pos = message_list[i][0].position[:7]
        if cup_idx_list[i] == cup_idx:
            print("got it!\n")
            waypoints += [point for point in message_list[i]]
            for j in range(len(message_list[i])-1):
                action_list.append([message_list[i][j+1].position[k]-message_list[i][j].position[k] for k in range(7)])
            # final pos have 0 as action
            action_list.append([0,0,0,0,0,0,0])
    waypoint_idx = list(range(len(waypoints)))
    random.shuffle(waypoint_idx)
    # shuffle waypoints and action_list
    print(len(waypoints))
    print(len(action_list))
    waypoints = [waypoints[i] for i in waypoint_idx]
    action_list = [action_list[i] for i in waypoint_idx]
    # print("each buffer has {} waypoints".format(len(waypoints)//len(buffers)))
    # num_split = len(waypoints) // len(buffers)
    # for i in range(len(buffers)):
    #     buffer = buffers[i]
    #     for j in range(num_split):
    #         obs = list(waypoints[top].position[:6])+cup_pos[cup_idx] + icecream_pos
    #         action = list(waypoints[top].effort[:6])
    #         buffer.add_sample(obs,action)
    #         top += 1
    # random.shuffle(waypoints)
    
    print("each buffer has {} waypoints".format(len(waypoints)//len(buffers)))
    num_split = len(waypoints) // len(buffers)
    for i in range(len(buffers)):
        buffer = buffers[i]
        for j in range(num_split):
            obs = list(waypoints[top].position[:7])+cup_pos[cup_idx] + icecream_pos
            # print(obs)
            # action = list(waypoints[top].effort[:6])
            action = action_list[top]
            buffer.add_sample(obs,action)
            top += 1

def split_traj_with_cup_pos_and_read_eef(buffers,message_list,cup_idx_list,cup_idx):
    obs_list = []
    action_list = []
    for i in range(len(message_list)):
        if cup_idx_list[i] == cup_idx:
            for j in range(len(message_list[i])-1):
                eef_pos = message_list[i][j]
                obs = list(eef_pos)+cup_pos[cup_idx] + icecream_pos
                action = [message_list[i][j+1][k]-eef_pos[k] for k in range(3)]
                obs_list.append(obs)
                action_list.append(action) 
            eef_pos = message_list[i][-1]
            obs = eef_pos+cup_pos[cup_idx] + icecream_pos
            action = [0,0,0]
    index = list(range(len(obs_list))) 
    random.shuffle(index)
    obs_list = [obs_list[i] for i in index]
    action_list = [action_list[i] for i in index]
    num_split = len(obs_list) // len(buffers)
    for i in range(len(buffers)):
        buffer = buffers[i]
        for j in range(num_split):
            obs = obs_list[i*num_split+j]
            # print(obs)
            action = action_list[i*num_split+j]
            buffer.add_sample(obs,action)
        


        

def device():
    if torch.cuda.is_available():
        return "cuda"
    else:
        print("WARNING: CUDA not available, running on CPU instead!")

def read_cup_index_from_csv(file_path:str):
    with open(file_path, 'r') as f:
        cup_index = []
        reader = csv.reader(f)
        cup_index = [int(row[2]) for row in reader]
    return cup_index

if __name__ == "__main__":
    # cup_idx = read_cup_index_from_csv("/home/hang/catkin_ws/src/ldf_with_progress/BC/participant_sheet.csv")
    # print(cup_idx)
    txt_file_list = find_all_txt_files_with_eef()
    read_txt_to_meaasages(txt_file_list)




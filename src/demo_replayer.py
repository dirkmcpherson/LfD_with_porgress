#!/usr/bin/env python
import rospy
import rosbag
from kortex_driver.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from kortex_driver.srv import *
from armpy import kortex_arm
import time
import numpy as np
class ArmReplayer:
    def __init__(self, folder_name = None):
        self.path = '/home/hang/catkin_ws/src/ldf_with_progress/bags/'
        if folder_name is not None:
            self.path = '/home/hang/catkin_ws/src/ldf_with_progress/bags/' + folder_name + '/'
        self.cnt = 0
        bag_name = self.path + str(self.cnt) + '.bag'
        #self.bag = rosbag.Bag(bag_name, 'r')
        
        self.joint_trajectory = ConstrainedJointAngles()
        self.arm = kortex_arm.Arm()
        self.arm.home_arm()
        self.arm.open_gripper()
        self.msg = []

    def read_bag(self, cnt=None):
        if cnt is None:
            cnt = self.cnt
        self.bag = rosbag.Bag(self.path + str(cnt) + ".bag", 'r')
        self.msg = []
        for topic, msg, t in self.bag.read_messages(topics=['/my_gen3_lite/joint_states']):
            temp = JointState()
            temp.header = msg.header
            temp.position = msg.position
            temp.velocity = msg.velocity
            temp.name = msg.name
            temp.effort = msg.effort
    
            self.msg.append(temp)

        # print((self.msg))
        return self.msg
    
    def replay_via_joint_state(self, msg = None):
        self.arm.home_arm()
        if msg is None:
            msg = self.msg
        waypoints = []
        for i in range(len(msg)):
            waypoints.append(msg[i].position[:6])
        self.arm.goto_joint_waypoints(waypoints, max_duration = 10)
        self.cnt += 1
        
    def replay_via_joint_state_with_gripper(self, msg = None):
        self.arm.home_arm()
        if msg is None:
            msg = self.msg
        waypoints = []
        for i in range(len(msg)):
            waypoints.append(msg[i].position)
        #print(waypoints)
        self.arm.goto_joint_gripper_waypoints(waypoints)
        self.cnt += 1
    
    def replay_with_progress_collect(self, frequency = 10, cnt = None, msg = None, auto = False):
        self.arm.home_arm()
        progresses = []
        scalars = []
        step = []
        if msg is None:
            msg = self.msg
        for i in range(0,frequency):
            
            waypoints = []
            for j in range(int(len(msg)/frequency) * i, min(len(msg), int(len(msg)/frequency) * (i + 1))):
                waypoints.append(msg[j].position)
            if i == frequency - 1:
                for j in range(int(len(msg)/frequency) * (i + 1), len(msg)):
                    waypoints.append(msg[j].position)
            self.arm.goto_joint_gripper_waypoints(waypoints)
            step.append(len(waypoints))
            random_number = np.random.rand()
            if auto:
                pass
            else:
                print("your replaying step is: ", i)
                if random_number < 0.5:
                    
                    
                    if len(progresses) > 0:
                        print("your last progress is: ", progresses[-1])
            
                    progress = input('Please input the progress: ')
                    progresses.append(progress)
                    scalar = input('Please input the scalar: ')
                    scalars.append(scalar)
                else:
                    scalar = input('Please input the scalar: ')
                    scalars.append(scalar)
                    if len(progresses) > 0:
                        print("your last progress is: ", progresses[-1])
                    progress = input('Please input the progress: ')
                    progresses.append(progress)
        
        print(progresses)
        print(step)
        #write progresses to file
        if cnt is None:
            cnt = self.cnt
        with open(self.path + str(cnt) + "_progress.txt", 'w') as f:
            for item in progresses:
                f.write("%s\n" % item)
            f.close()
        with open(self.path + str(cnt) + "_scalar.txt", 'w') as f:
            for item in scalars:
                f.write("%s\n" % item)
            f.close()
        with open(self.path + str(cnt) + "_step.txt", 'w') as f:
            for item in step:
                f.write("%s\n" % item)
            f.close()
        self.cnt += 1

    def replay_via_joint_state_with_speed(self, msg = None):
        self.arm.home_arm()
        if msg is None:
            msg = self.msg
        for i in range(len(msg)):
            self.arm.joint_velocity_command(msg[i].velocity[:6], 1 / 100)

    def out_put_positions(self, msg = None):
        if msg is None:
            msg = self.msg
        poses = []
        
        for i in range(len(msg)):
            temp = JointState()

            temp.position = msg[i].position
            temp.name = msg[i].name
            print(temp)
            poses.append(self.arm.get_fk(temp))
            break

        return poses
    def close_bag(self):
        self.bag.close()
    def write_poses_to_file(self, poses, cnt = None):
        if cnt is None:
            cnt = self.cnt
        with open(self.path + str(cnt) + "_poses.txt", 'w') as f:
            for item in poses:
                #print(item.pose.position.x, item.pose.position.y, item.pose.position.z)
                f.write("%s %s %s\n" % (item.pose.position.x, item.pose.position.y, item.pose.position.z))
            f.close()

if __name__ == '__main__':
    rospy.init_node('arm_replayer', anonymous=True)
    #folder = input('Please input the folder name: ')
    for i in range(25, 40):
        folder = "user_" + str(i)
    #folder = "user_39"
        replayer = ArmReplayer(folder)
    #bag = input('Please input the bag number: ')
        bag = 0
        replayer.read_bag(bag)

        poses = replayer.out_put_positions()
        #replayer.write_poses_to_file(poses)
        print(poses)


    # # now = time.time()
    # # replayer.replay_via_joint_state_with_speed()
    # # print(time.time() - now)


    # now = time.time()
    # replayer.replay_via_joint_state_with_gripper()
    # print(time.time() - now)

    # now = time.time()
    # replayer.replay_via_joint_state()
    # print(time.time() - now)

    # now = time.time()
    # replayer.replay_with_progress_collect()
    # print(time.time() - now)

    # #replayer.play_trajectory()
        replayer.close_bag()
    #rospy.spin()

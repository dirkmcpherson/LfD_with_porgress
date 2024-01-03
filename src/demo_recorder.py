#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import JointState
from pynput import keyboard
import time
from armpy import kortex_arm
from geometry_msgs.msg import Pose


class ArmRecorder:
    def __init__(self, folder_name = None):
        self.arm = kortex_arm.Arm()
        self.arm.home_arm()
        self.arm.open_gripper()
        # time.sleep(2)
        # self.arm.close_gripper()
        self.cnt = 0
        self.path = '/home/hang/catkin_ws/src/ldf_with_progress/bags/'
        if folder_name is not None:
            self.path = '/home/hang/catkin_ws/src/ldf_with_progress/bags/' + folder_name + '/'
            #create folder if not exist
            import os
            if not os.path.exists(self.path):
                os.makedirs(self.path)
        
        bag_name = self.path + str(self.cnt) + '.bag'
        self.bag = rosbag.Bag(bag_name, 'w')
        self.is_recording = False
        self.last_record_time = time.time()
        self.record_interval = 1.0 / 8  # Interval for 5 Hz recording
        self.lastest_obj_pose = None
       # self.obj_pose_sub = rospy.Subscriber("obj_pose", Pose, self.obj_pose_callback)
        self.joint_state_sub = rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_state_callback)


        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 's':
                self.start_recording()
            elif key.char == 'p':
                self.stop_recording()
            elif key.char == 'e':
                self.end_script()
            elif key.char == 'n':
                self.next_recording()
            elif key.char == 'r':
                self.reset_recording()
        except AttributeError:
            pass
    def reset_recording(self):
        self.stop_recording()
        self.close_bag()
        rospy.sleep(10)
        rospy.loginfo("Re-Record the bag.")
        self.arm.home_arm()
        bag_name =  self.path + str(self.cnt) + '.bag'
        self.bag = rosbag.Bag(bag_name, 'w') 
         
    def next_recording(self):
        self.stop_recording()
        self.close_bag()
        rospy.sleep(10)
        rospy.loginfo("Record a new bag.")
        #time.sleep(1)
        self.arm.home_arm()
        self.cnt += 1  # Increment the counter for a new file
        bag_name = self.path  + str(self.cnt) + '.bag'
        self.bag = rosbag.Bag(bag_name, 'w') 
        print("New bag created.")
        

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            rospy.loginfo("Recording started.")
            print("Recording started.")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            rospy.loginfo("Recording stopped.")
            print("Recording stopped.")
            #self.close_bag()

    def end_script(self):
        self.close_bag()
        self.listener.stop()
        rospy.signal_shutdown("Script ended by user.")
        print("Script ended by user.")
    def obj_pose_callback(self, msg):
            self.latest_obj_pose = msg

    def joint_state_callback(self, msg):
        if self.is_recording and time.time() - self.last_record_time >= self.record_interval:
            try:
                self.last_record_time = time.time()
                # print(msg)
                self.bag.write("/my_gen3_lite/joint_states", msg)
                # if self.latest_obj_pose:
                #     self.bag.write("obj_pose", self.latest_obj_pose)
            except ValueError as e:
                rospy.logwarn("Error writing to bag file: " + str(e))
                # Additional handling if nesecessary


    def close_bag(self):
        time.sleep(1)
        self.bag.close()
        rospy.loginfo("Bag file saved.")

if __name__ == '__main__':
    rospy.init_node('arm_recorder', anonymous=True, disable_signals=True)
    rospy.Rate(30)  # This rate is not used for recording, only for rospy spin 
    #folder = input('Please input the folder name: ')
    folder = "user3"
    recorder = ArmRecorder(folder)
    rospy.spin()

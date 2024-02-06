#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import JointState
from pynput import keyboard
import time
from armpy import kortex_arm
from geometry_msgs.msg import Pose
import cv2
import numpy as np

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
            else:
                now = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time()))
                self.path = '/home/hang/catkin_ws/src/ldf_with_progress/bags/' + folder_name + now + '/'
        
        bag_name = self.path + str(self.cnt) + '.bag'
        self.bag = rosbag.Bag(bag_name, 'w')
        self.is_recording = False
        self.last_record_time = time.time()
        self.record_interval = 1.0 / 5  # Interval for 5 Hz recording
        # self.lastest_obj_pose = Pose()
        # self.lastest_obj_pose.position.x = 0
        # self.lastest_obj_pose.position.y = 0
        # self.latest_obj_pose = []
        # self.obj_pose_sub = rospy.Subscriber("/obj_pose", Pose, self.obj_pose_callback)
        self.joint_state_sub = rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_state_callback)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.video_writer = None
        if not self.cap.isOpened():
            rospy.logerr("Error: Camera is not opened.")
            exit()
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
    
    def save_obj_pose(self):
        if self.latest_obj_pose:
            latest_obj_pose_x = np.array([item[0] for item in self.latest_obj_pose])
            latest_obj_pose_y = np.array([item[1] for item in self.latest_obj_pose])
            
            latest_obj_pose_x, latest_obj_pose_y = get_accurate_obj_pose()
            with open("/home/hang/catkin_ws/src/ldf_with_progress/src/obj_pos/obj_pos.txt", 'w') as f:
                f.write("%s %s\n" % (latest_obj_pose_x, latest_obj_pose_y))
                f.close()
            rospy.loginfo("Object pose received.")
        def get_accurate_obj_pose():
            latest_obj_pose_x = np.array([item[0] for item in self.latest_obj_pose])
            latest_obj_pose_y = np.array([item[1] for item in self.latest_obj_pose])
            
            Q1 = np.percentile(latest_obj_pose_x, 25)
            Q3 = np.percentile(latest_obj_pose_x, 75)
            
            IQR = Q3 - Q1
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR
            latest_obj_pose_x = latest_obj_pose_x[(latest_obj_pose_x > lower_bound) & (latest_obj_pose_x < upper_bound)]
            
            Q1 = np.percentile(latest_obj_pose_y, 25)
            Q3 = np.percentile(latest_obj_pose_y, 75)
            
            IQR = Q3 - Q1
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR
            latest_obj_pose_y = latest_obj_pose_y[(latest_obj_pose_y > lower_bound) & (latest_obj_pose_y < upper_bound)]
            
            return np.mean(latest_obj_pose_x), np.mean(latest_obj_pose_y)
    def start_recording(self):
        if not self.is_recording:
            # for i in range(3):
            #     rospy.sleep(1)
            #     rospy.loginfo("Recording will start in " + str(3 - i) + " seconds.")
            self.is_recording = True
            rospy.loginfo("Recording started.")
            print("Recording started.")
            # self.save_obj_pose()
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.path + str(self.cnt) + '.avi', fourcc, 20.0, (640, 480))
            if not self.video_writer.isOpened():
                rospy.logerr("Error: Video writer not opened.")
                exit()


    def record_video_frame(self):
        if self.is_recording and self.video_writer is not None:
            ret, frame = self.cap.read()
            if ret:
                self.video_writer.write(frame)
            else:
                rospy.logwarn("Error: Frame capture failed.")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            rospy.loginfo("Recording stopped.")
            print("Recording stopped.")
            self.latest_obj_pose = []
            #self.close_bag()
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None

    def end_script(self):
        self.close_bag()
        self.listener.stop()
        rospy.signal_shutdown("Script ended by user.")
        print("Script ended by user.")
        self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()


    def obj_pose_callback(self, msg):
            self.latest_obj_pose = msg
            self.latest_obj_pose.append((msg.point.x, msg.point.y))
            
            

    def joint_state_callback(self, msg):
        if self.is_recording and time.time() - self.last_record_time >= self.record_interval:
            try:
                self.last_record_time = time.time()
                # print(msg)
                self.bag.write("/my_gen3_lite/joint_states", msg)
                # if self.lastest_obj_pose:
                #     self.bag.write("obj_pose", self.lastest_obj_pose)
            except ValueError as e:
                rospy.logwarn("Error writing to bag file: " + str(e))
                # Additional handling if nesecessary


    def close_bag(self):
        time.sleep(3)
        self.bag.close()
        rospy.loginfo("Bag file saved.")

if __name__ == '__main__':
    rospy.init_node('arm_recorder', anonymous=True, disable_signals=True)
    rospy.Rate(30)  # This rate is not used for recording, only for rospy spin 
    #folder = input('Please input the folder name: ')
    
    folder = "user_25"
    recorder = ArmRecorder(folder)
    recorder.record_video_frame()
    while not rospy.is_shutdown():
        recorder.record_video_frame()
        rospy.sleep(0.05)
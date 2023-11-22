#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import JointState
from pynput import keyboard
import time
from armpy import kortex_arm

class ArmRecorder:
    def __init__(self, bag_name):
        self.arm = kortex_arm.Arm()
        self.arm.home_arm()
        # self.arm.open_gripper()
        # time.sleep(2)
        # self.arm.close_gripper()
        self.bag = rosbag.Bag(bag_name, 'w')
        self.is_recording = False
        self.last_record_time = time.time()
        self.record_interval = 1.0 / 3  # Interval for 5 Hz recording
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
        except AttributeError:
            pass

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            rospy.loginfo("Recording started.")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            rospy.loginfo("Recording stopped.")

    def end_script(self):
        self.close_bag()
        self.listener.stop()
        rospy.signal_shutdown("Script ended by user.")

    def joint_state_callback(self, msg):
        if self.is_recording and time.time() - self.last_record_time >= self.record_interval:
            self.last_record_time = time.time()
            print(msg)
            self.bag.write("/my_gen3_lite/joint_states", msg)

    def close_bag(self):
        self.bag.close()
        rospy.loginfo("Bag file saved.")

if __name__ == '__main__':
    rospy.init_node('arm_recorder', anonymous=True, disable_signals=True)
    rospy.Rate(10)  # This rate is not used for recording, only for rospy spin loop
    recorder = ArmRecorder('arm_movement.bag')
    rospy.spin()

#!/usr/bin/env python
import rospy
import rosbag
from kortex_driver.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from kortex_driver.srv import *
from armpy import kortex_arm
import time
class ArmReplayer:
    def __init__(self, bag_name):
        self.bag = rosbag.Bag(bag_name, 'r')
        self.joint_trajectory = ConstrainedJointAngles()
        self.arm = kortex_arm.Arm()
        self.arm.home_arm()

    # def read_bag(self):
    #     play_joint_traj = rospy.ServiceProxy('/my_gen3_lite/base/play_pre_computed_joint_trajectory', PlayPreComputedJointTrajectory)
    #     #cja = PlayJointTrajectoryRequest()
    #     #cja = ConstrainedJointAngles()
    #     traj = PlayPreComputedJointTrajectoryRequest()
    #     #print(cja)
    #     cnt = 0
    #     print(self.bag.get_type_and_topic_info())
    #     for topic, msg, t in self.bag.read_messages(topics=['/my_gen3_lite/joint_states']):
    #         #print(msg)
            
    #         if cnt == 0:
    #             self.arm.goto_joint_pose(msg.position[:6])
            
    #         if isinstance(msg, JointState) or 1:
    #             point = PreComputedJointTrajectoryElement()
    #             point.joint_angles = msg.position[:6]
    #             #point.joint_angles = [x * 180 / 3.1415926 for x in point.joint_angles]
    #             point.joint_speeds = msg.velocity[:6]
    #             #point.joint_speeds = [x * 180 / 3.1415926 for x in point.joint_speeds]  
    #             point.joint_accelerations = msg.effort[:6]
    #             point.time_from_start = 1 / 5 * cnt
    #             #print(point)
    #             traj.input.trajectory_elements.append(point)
    #         cnt = cnt + 1
    #     #traj.mode = 0
    #     print(traj)
    #     play_joint_traj(traj)

            # cja.constraint.type = 0
            # cja.constraint.value = 0.0 
    # def read_bag(self):
    #     play_joint_traj = rospy.ServiceProxy('/my_gen3_lite/base/play_joint_trajectory', PlayJointTrajectory)
    #     #cja = PlayJointTrajectoryRequest()
    #     #cja = ConstrainedJointAngles()
    #     trj = []
    #     #print(cja)
    #     cnt = 0
    #     for topic, msg, t in self.bag.read_messages(topics=['/my_gen3_lite/joint_states']):
    #         cja = PlayJointTrajectoryRequest()
    #         #print(msg)
    #         if cnt == 0:
    #             self.arm.goto_joint_pose(msg.position[:6])
    #         cnt += 1
    #         if isinstance(msg, JointState) or 1:
    #             for i in range(6): 
    #                 joint = JointAngle()
    #                 joint.joint_identifier = i 
    #                 joint.value = msg.position[i] * 180 / 3.1415926
    #                 cja.input.joint_angles.joint_angles.append(joint)
    #         # cja.constraint.type = 0
    #         # cja.constraint.value = 0.0
    #         play_joint_traj(cja) 
    #         time.sleep(1)   
    #         print( cja)  
    def read_bag(self):
        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/my_gen3_lite/joint_states']):
            #print(msg)
            if cnt == 0:
                self.arm.goto_joint_pose(msg.position[:6])
            cnt += 1
            self.arm.joint_velocity_command(msg.velocity[:6], 1 / 10)
     
    # def read_bag(self):
    #     waypoints = []
    #     for topic, msg, t in self.bag.read_messages(topics=['/my_gen3_lite/joint_states']):
    #         #print("hhhh")
    #         #print(msg)
    #         if isinstance(msg, JointState) or 1:
    #             waypoints.append(msg.position[:6])
    #     self.arm.goto_joint_waypoints(waypoints)

    #     print(self.joint_trajectory)
    # def play_trajectory(self):
    #     print("start_replay1")
    #     rospy.wait_for_service('/my_gen3_lite/base/play_joint_trajectory')
    #     print("start_replay2")
    #     try:
    #         play_joint_traj = rospy.ServiceProxy('/my_gen3_lite/base/play_joint_trajectory', PlayJointTrajectory)
    #         print("start_replay3")
    #         resp = play_joint_traj(self.joint_trajectory)
    #         rospy.loginfo("Trajectory sent. Service response: %s", resp)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s", e)

    def close_bag(self):
        self.bag.close()

if __name__ == '__main__':
    rospy.init_node('arm_replayer', anonymous=True)
    replayer = ArmReplayer('/home/hang/catkin_ws/src/ldf_with_progress/bags/arm_movement0.bag')
    now = time.time()
    replayer.read_bag()
    print(time.time() - now)
    #replayer.play_trajectory()
    replayer.close_bag()
    #rospy.spin()

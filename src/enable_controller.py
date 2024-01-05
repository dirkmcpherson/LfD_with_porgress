#!/usr/bin/env python3

import asyncio
import functools
import rospy

import sensor_msgs.msg

import study_runner
from study_runner.frames.logging import LoggingFrame, RunLogging
import study_runner.frames.loggers
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
import tkinter
import geometry_msgs.msg
import threading
from armpy import kortex_arm
import geometry_msgs.msg
from kortex_driver.msg import *
import time


height = 100
# TODO: do something smarter with this
class AsyncSubscriber:
    def __init__(self, topic, type, loop=None, queue_size=0):
        self._subscriber = rospy.Subscriber(topic, type, self._cb)
        self._queue = asyncio.Queue(queue_size)
        self._loop = loop or asyncio.get_running_loop()
    def _cb(self, msg):
        self._loop.call_soon_threadsafe(functools.partial(self._queue.put_nowait, msg))

    def get(self):
        return self._queue.get()
    def close(self):
        self._subscriber.unregister()

def check_limit(limit_check):
    global height 
    robot = kortex_arm.Arm("my_gen3_lite")
    temp = 10
    cartesian_vel_publisher = rospy.Publisher(
            f"/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1, latch=True)
    while rospy.is_shutdown() is False:
        z = robot.get_eef_pose(quaternion=False)[2]
        height = z
        if z < limit_check + 0.001:
            #print(z, temp)
            if temp - z > 0.001:
                cartesian_command = TwistCommand()
                cartesian_vel_publisher.publish(cartesian_command)
                temp = z
                time.sleep(1)
            if z - temp > 0.00:
                temp = z
                time.sleep(0.1)
        else:
            temp = z
            # Continue with your existing logic or do nothing
            time.sleep(0.1)
            pass

async def run_teleop(config, status_cb):
    

    plugin, profile = get_teleop_info(config)
    print(plugin, profile)
    sub = AsyncSubscriber("/joy", sensor_msgs.msg.Joy)
    limit_check = 0.03
    limit_thread = threading.Thread(target=check_limit, args=(limit_check,))
    limit_thread.start()

    with RunLogging(config):
        try:
            while not rospy.is_shutdown():
                #print("waiting for msg")
                msg = await sub.get()
                #print("got msg:  " , msg)
                cmd = profile.process_input(msg)
                plugin.do_command(cmd, limit_check, height)
        finally:
            sub.close()

def main():
    rospy.init_node("collect_teleop_data", anonymous=True)
    print("init")
    root = tkinter.Tk()
    print("got tk")
    runner = study_runner.StudyRunner(root, run_teleop)
    print("build basic runner")
    runner.add_config_frame(TeleopConfigFrame, "Teleoperation")
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    #logging_frame.add_logger_frame(study_runner.frames.loggers.rosbag_recorder.RosbagRecorderConfigFrame, side="right")
    #logging_frame.add_logger(study_runner.frames.loggers.rosbag_recorder.ROSBAG_RECORDER_CONFIG_NAME)
    print("running...")
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()


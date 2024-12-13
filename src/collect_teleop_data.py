#!/usr/bin/env python3

import asyncio
import functools
import rospy

import sensor_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kortex_driver.msg import BaseCyclic_Feedback

import study_runner
from study_runner.frames.logging import LoggingFrame, RunLogging
import study_runner.frames.loggers
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from study_runner.frames.loggers.rosbag_recorder import get_rosbag_recorder
import tkinter
import yaml
import os
import time

class RemoteStart:
    def __init__(self, log_dir, topic):
        self.log_dir = log_dir
        self.topic = topic
        self.pub = rospy.Publisher(topic, String, latch=True, queue_size=1)

    def start(self):
        self.pub.publish(self.log_dir)

    def stop(self):
        self.pub.publish('')


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

async def run_teleop(config, status_cb):

    plugin, profile = get_teleop_info(config)
    sub = AsyncSubscriber("/joy", sensor_msgs.msg.Joy, queue_size=1)
    state_sub = AsyncSubscriber("/my_gen3_lite/base_feedback", BaseCyclic_Feedback, queue_size=1)
    # uid_pub = rospy.Publisher('/uid_data_dir', String, latch=True, queue_size=1)
    with RunLogging(config):
        # uid_pub.publish(config['logging']['data_dir'])
        try:
            # print(f'{plugin=}, {profile=}, {config=}')
            t0 = time.time(); loops = 0
            print(f"Starting loop")
            while not rospy.is_shutdown():
                msg = await sub.get()
                baseCyclic_Feedback = await state_sub.get()
                cmd = profile.process_input(msg)
                plugin.do_command(cmd, current_state=baseCyclic_Feedback)

                loops +=1 
                if loops % 100 == 0: print(f"Heartbeat running at {1 / ((time.time() - t0) / loops):1.2f} HZ"); loops = 0; t0 = time.time()

        finally:
            # uid_pub.publish('')
            sub.close()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config_file', default=None)
    args = parser.parse_args()

    rospy.init_node("collect_teleop_data", anonymous=True)
    print("init")
    root = tkinter.Tk()
    print("got tk")

        # Define initial configurations
    default_config = {
        'teleop': {
            'plugin': 'gen3_lite',
        }
    }
        # Load the control mode from the YAML file
    mode_file_path = '/home/j/workspace/LfD_with_porgress/config/js.yaml'
    try:
        with open(mode_file_path, 'r') as f:
            mode_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading mode file: {e}")
        mode_data = {}
    else:
        # Add 'filename' key to mode_data
        mode_data['filename'] = os.path.basename(mode_file_path)

        # Include the mode in the default configuration
        default_config['teleop']['modes'] = [mode_data]
        default_config['teleop']['plugin_args'] = mode_data['plugin_args'] if 'plugin_args' in mode_data else {}
        for k,v in default_config.items():
            print(f"\tLfD: {k} {v}")

    # Initialize the runner with initial configurations
    # runner = study_runner.StudyRunner(
    #     root, run_teleop, initial_config_file=initial_config
    # )
    print("build basic runner")
    runner = study_runner.StudyRunner(root, run_teleop, initial_config_file=mode_file_path, default_config = default_config)
    print("built basic runner")
    runner.add_config_frame(TeleopConfigFrame, "Teleoperation")
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    # bagrecorder = get_rosbag_recorder()
    # print(f'{bagrecorder=}')
    logging_frame.add_logger_frame(study_runner.frames.loggers.rosbag_recorder.RosbagRecorderConfigFrame, side="right")
    logging_frame.add_logger('remote_start', lambda logdir, cfg: RemoteStart(logdir, '/uid_data_dir'))

    # logging_frame.add_logger(study_runner.frames.loggers.rosbag_                                                                                                                                                                                                                                           recorder.ROSBAG_RECORDER_CONFIG_NAME)
    print("running...")
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()


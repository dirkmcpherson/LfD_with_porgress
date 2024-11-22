#!/usr/bin/env python3

import asyncio
import functools
import rospy

import sensor_msgs.msg
from std_msgs.msg import String

import study_runner
from study_runner.frames.logging import LoggingFrame, RunLogging
import study_runner.frames.loggers
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from study_runner.frames.loggers.rosbag_recorder import get_rosbag_recorder
import tkinter

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
    print(f'{plugin=}, {profile=}, {config=}')
    sub = AsyncSubscriber("/joy", sensor_msgs.msg.Joy)
    # uid_pub = rospy.Publisher('/uid_data_dir', String, latch=True, queue_size=1)
    with RunLogging(config):
        # uid_pub.publish(config['logging']['data_dir'])
        try:
            while not rospy.is_shutdown():
                msg = await sub.get()
                cmd = profile.process_input(msg)
                print(cmd)
                plugin.do_command(cmd)
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
    runner = study_runner.StudyRunner(root, run_teleop, initial_config_file=args.config_file)
    print("build basic runner")
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


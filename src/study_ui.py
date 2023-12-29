import demo_recorder
import rospy
import demo_replayer
from pynput import keyboard

def on_press(key):
    try:
        if key.char == 's':
            demo_recorder.start_recording()
        elif key.char == 'p':
            demo_recorder.stop_recording()
        # elif key.char == 'e':
        #     demo_recorder.end_script()
        elif key.char == 'n':
            demo_recorder.next_recording()
        elif key.char == 'r':
            demo_recorder.reset_recording()
        elif key.char == '1':
            demo_replayer.replay_via_joint_state()
        elif key.char == '2':
            demo_replayer.replay_with_progress_collect()
    except AttributeError:
        pass


if __name__ == '__main__':
    rospy.init_node('study_runner', anonymous=True, disable_signals=True)
    demo_recorder = demo_recorder.ArmRecorder()
    demo_replayer = demo_replayer.ArmReplayer()
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    rospy.spin()
    
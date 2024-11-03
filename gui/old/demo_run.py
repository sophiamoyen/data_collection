import sys
import signal
import traceback

import rospy
from demo_functions import RecordLeft


def main(path_to_logs):
    try:
        def close(signum, frame):
            record.close()
            sys.exit(0)

        signal.signal(signal.SIGTSTP, close)
        signal.signal(signal.SIGINT, close)

        rospy.init_node('record_left')
        record = RecordLeft(path_to_logs)

    except Exception as e:
        print(e)
        traceback.print_exc(e)
    finally:
        record.close()
        return 0

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please provide a name to save the recording to. \nCall "python wbc_teleop_collect_demo.py <path_to_logs>"')
        sys.exit(0)

    path_to_logs= str(sys.argv[1])
    exit(main(path_to_logs))
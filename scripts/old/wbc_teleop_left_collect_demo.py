# Author: Sophie Lueth
# Date: July 2024

import sys
import signal
import traceback

import rospy
from record_left import RecordLeft


def main(path_to_logs, collect):
    try:
        def close(signum, frame):
            record.close()
            sys.exit(0)

        signal.signal(signal.SIGTSTP, close)
        signal.signal(signal.SIGINT, close)

        rospy.init_node('record_left')
        record = RecordLeft(path_to_logs, collect=collect)
        record.run()

    except Exception as e:
        print(e)
        traceback.print_exc(e)
    finally:
        record.close()
        return 0

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Please provide a name to save the recording to and specify whether you want to record demos of tune impedance parameters. \nCall "python wbc_teleop_collect_demo.py <path_to_logs> <record/tune>"')
        sys.exit(0)
    
    record_var = str(sys.argv[2])
    
    if record_var == 'record':
        collect = True
    elif record_var == 'tune':
        collect = False
    else:
        print('Please provide a name to save the recording to. Call "python wbc_teleop_collect_demo.py <filename> <record/tune>"')
        sys.exit(0)

    path_to_logs= str(sys.argv[1])
    exit(main(path_to_logs, collect))
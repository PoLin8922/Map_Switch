#!/usr/bin/env python

import rospy
import subprocess
import signal
import sys
import numpy as np
import os
import sys
import termios
from multimap_server.srv import MapFilePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

def press_any_key_exit(msg):
    fd = sys.stdin.fileno()
    old_ttyinfo = termios.tcgetattr(fd)
    new_ttyinfo = old_ttyinfo[:]
    new_ttyinfo[3] &= ~termios.ICANON
    new_ttyinfo[3] &= ~termios.ECHO
    print(msg)
    sys.stdout.flush()
    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
    global flag
    try:
        os.read(fd, 7)
    except:
        # quit()
        # sys.exit()
        flag = False
        pass
    termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)

if __name__ == "__main__":
    global flag
    flag = True
    press_any_key_exit("Press enter to sent initial pose...")
    print('yahahaha!!')
    press_any_key_exit("Press enter to sent initial pose...")
    if flag == True:
        print('ohhhhhhh!!')
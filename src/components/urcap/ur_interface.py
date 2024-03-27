#!/usr/bin/env python3

""" 
    This python file is binded to a C++ dynamic library to call its functions at run time using interpreter
    The C++ .so must be installed in the /usr/local/lib directory by building it seperately
    The C++ code interfaces with the URCap API for the Husky robot to give it remote commands
    The python code does nothing but make the code callable from other python scripts
"""

import ctypes
import argparse

SYS_LIB_PATH            = "/usr/local/lib/"
SO_FILE_NAME            = "liburclient.so"
UR_CLIENT               = ctypes.CDLL(SYS_LIB_PATH + SO_FILE_NAME)

def play_button():
    UR_CLIENT.pressPlayButton()
def release_brakes():
    UR_CLIENT.releaseBrakes()
def unlock_protective_stop():
    UR_CLIENT.unlockProtectiveStop()
# def init_robot():
#     UR_CLIENT.init_robot()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--action', type=str)
    args = parser.parse_args()

    if args.action == 'r':
        release_brakes()
    elif args.action == 'u':
        unlock_protective_stop()
    elif args.action == 'p':
        play_button()
#!/usr/bin/env python3

""" 
    This python file is binded to a C++ dynamic library to call its functions at run time using interpreter
    The C++ .so must be installed in the /usr/local/lib directory by building it seperately
    The C++ code interfaces with the URCap API for the Husky robot to give it remote commands
    The python code does nothing but make the code callable from other python scripts
"""

import ctypes

SYS_LIB_PATH            = "/usr/local/lib/"
SO_FILE_NAME            = "liburclient.so"
UR_CLIENT               = ctypes.CDLL(SYS_LIB_PATH + SO_FILE_NAME)

def play_button():
    UR_CLIENT.press_play_button()
def release_brakes():
    UR_CLIENT.release_brakes()
def unlock_protective_stop():
    UR_CLIENT.unlock_protective_stop()
def init_robot():
    UR_CLIENT.init_robot()

if __name__ == "__main__":
    release_brakes()
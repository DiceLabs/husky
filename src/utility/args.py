import argparse
import sys
from dexterity import Dexterity

def convert_dext_str_to_enum(arg_value: str):
    LEFT_FLAG = "l"
    RIGHT_FLAG = "r"
    INVALID_DEXT_ARG_MSG = "Invalid value for dexterity. Use 'l' for left or 'r' for right."
    flag_converter = {LEFT_FLAG: Dexterity.LEFT, RIGHT_FLAG: Dexterity.RIGHT}
    if arg_value.lower() in flag_converter:
        return flag_converter[arg_value.lower()]
    else:
        raise argparse.ArgumentTypeError(INVALID_DEXT_ARG_MSG)

def get_cmd_args():
    FLAG_ID = '-d'
    FLAG_ATTR = '--dexterity'
    FLAG_HELP = "indicate dexterity of ur5e_arm with d flag followed by l for left and r for right"
    NO_FLAG_MSG = "No command line argument passed. Please provide an argument."
    MIN_ARG_COUNT = 2

    if len(sys.argv) < MIN_ARG_COUNT:
        raise Exception(NO_FLAG_MSG)
    parser = argparse.ArgumentParser(description=FLAG_HELP)
    parser.add_argument(FLAG_ID, FLAG_ATTR)
    return parser.parse_args()
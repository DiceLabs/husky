import pyrealsense2 as rs
from multiprocessing import Process

def get_serial_devs():
    serial_devs = []
    ctx = rs.context()
    if len(ctx.devices) > 0:
        for d in ctx.devices:
            print ('Found device: ', \
                    d.get_info(rs.camera_info.name), ' ', \
                    d.get_info(rs.camera_info.serial_number)) 
            serial_devs.append(d.get_info(rs.camera_info.serial_number))
    else:
        print("No Intel Device connected")
    return serial_devs


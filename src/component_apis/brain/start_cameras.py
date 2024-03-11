from multiprocessing import Process
from dataclasses import dataclass
from camera import CameraNode

@dataclass 
class CameraInfo():
    depth :float
    x_dist:float
    y_dist:float

'''
    Dual Stream Camera Info
'''

def main():
    camera_info = CameraInfo()
    camera = CameraNode(camera_info)
    camera_process = Process(target=camera.camera_loop)
    camera_process.start()
    camera_process.join()


if __name__ == '__main__':
    main()
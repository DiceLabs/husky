#!/usr/bin/env python3

from multiprocessing import Process
from camera import CameraNode
from config import CAMERA_IDS
from services import ServiceNames, ServicePorts
from server import start_server

def start_camera_server(camera_id, name, port):
    camera = CameraNode(camera_id)
    start_server(name=name, port=port, callback=camera.camera_loop)

def start_camera(camera_id, service_name, service_port):
    process = Process(target=start_camera_server, args=(camera_id, service_name, service_port))
    process.start()
    process.join()

if __name__ == "__main__":
    start_camera(CAMERA_IDS.LEFT,  ServiceNames.LEFT_CAMERA,  ServicePorts[ServiceNames.LEFT_CAMERA])
    start_camera(CAMERA_IDS.RIGHT, ServiceNames.RIGHT_CAMERA, ServicePorts[ServiceNames.RIGHT_CAMERA])

#!/usr/bin/env python3

from multiprocessing import Process
from camera import CameraNode
from config import CAMERA_IDS
from services import ServiceNames, ServicePorts
from server import start_server

def start_camera_server(name, port, callback):
    start_server(name=name, port=port, callback=callback)

def start_camera(camera_id, service_name, service_port):
    camera = CameraNode(camera_id)
    process = Process(target=start_camera_server, args=(service_name, service_port, camera.camera_loop))
    process.start()
    process.join()

if __name__ == "__main__":
    # start_camera(CAMERA_IDS.LEFT,  Services.LEFT_CAMERA,  SERVICES[Services.LEFT_CAMERA])
    start_camera(CAMERA_IDS.RIGHT, ServiceNames.RIGHT_CAMERA, ServicePorts[ServiceNames.RIGHT_CAMERA])

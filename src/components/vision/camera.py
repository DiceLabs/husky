#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from connect import get_serial_devs
from timer import Timer
from multiprocessing import Process
from dataclasses import dataclass

UPDATE_RATE = 45    # Hz

@dataclass
class CameraInfo():
    depth: float
    distx: float
    disty: float

class CameraNode():
    COLOR_DICT = {
        "Red": (255, 0, 0),
        "Green": (0, 255, 0),
        "Blue": (0, 0, 255),
        "Yellow": (255, 255, 0),
        "Cyan": (0, 255, 255),
        "Magenta": (255, 0, 255),
        "White": (255, 255, 255),
        "Black": (0, 0, 0),
    }

    def __init__(self, serial_number, model_name='best.pt'):
        self.pipeline = self.camera_init(serial_number)
        self.model = YOLO(model_name)

    def camera_init(self, serial_number):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_number)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        return pipeline

    def find_closest_color(self, rgb_pixel):
        min_distance = float('inf')
        closest_color = ""
        for color_name, color_rgb in self.COLOR_DICT.items():
            distance = sum((c1 - c2) ** 2 for c1, c2 in zip(rgb_pixel, color_rgb)) ** 0.5
            if distance < min_distance:
                min_distance = distance
                closest_color = color_name
        return closest_color

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        return frames, color_frame, depth_frame

    def align_frames(self, frames):
        align = rs.align(rs.stream.color)
        aligned_frames = align.process(frames)
        return aligned_frames.get_depth_frame(), aligned_frames.get_color_frame()

    def process_detection(self, boxes, img, depth_frame):
        for box in boxes:
            bbox_coords = box.xyxy.squeeze().cpu().numpy()
            self.draw_bounding_box(img, bbox_coords)
            self.calculate_and_display_depth(img, bbox_coords, depth_frame)

    def draw_bounding_box(self, img, bbox_coords):
        x1, y1, x2, y2 = map(int, bbox_coords)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

    def calculate_and_display_depth(self, img, bbox_coords, depth_frame):
        x1, y1, x2, y2 = map(int, bbox_coords)
        mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
        depth = depth_frame.get_distance(mid_x, mid_y)
        depth_text = f"Depth: {depth:.2f}m"
        cv2.putText(img, depth_text, (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def get_camera_info(self, img, results, depth_frame):
        img_width = img.shape[1]  
        img_height = img.shape[0] 
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        intrinsics = depth_profile.get_intrinsics()
        fx = intrinsics.fx  
        fy = intrinsics.fy  
        for result in results:
            boxes = result.boxes 
            # You can use boxes.conf also to get the max with arg max and get ride of the other one, there proably would need to be a try except clause to handle nothing found 
            for box in boxes:
                bbox_coords = box.xyxy.squeeze().cpu().numpy()  
                x1, y1, x2, y2 = bbox_coords
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                image_center_x = img_width // 2
                image_center_y = img_height // 2
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2
                depth = depth_frame.get_distance(mid_x, mid_y)
                dx = mid_x - image_center_x
                dy = mid_y - image_center_y
                real_dx = (dx* 0.7 * depth) / fx
                real_dy = (dy* 0.7 * depth) / fy
                return depth, real_dx, real_dy


    def camera_loop(self, visualmode=False):
        WINDOW_NAME = 'RealSense Camera'
        frames, color_frame, depth_frame = self.get_frames()
        if not color_frame:
            return
        aligned_depth_frame, aligned_color_frame = self.align_frames(frames)
        img = np.asanyarray(aligned_color_frame.get_data())
        results = self.model(img, stream=True, conf=0.4)# you can change conf value, this gets rid of low prob predicitons, in a sterile environment this should be good enough
        if visualmode:
            cv2.imshow(WINDOW_NAME, img)
        
        try: 
            depth, dx, dy = self.get_camera_info(img=img, results=results, depth_frame=depth_frame)
            return depth, dx, dy
        except TypeError: pass
    
    def cleanup(self):
        USER_EXIT_MSG = 'User requested exit.'
        self.pipeline.stop(USER_EXIT_MSG)
        cv2.destroyAllWindows()
        
def camera_wrap(serial_number):
    freq = 45
    camera = CameraNode(serial_number)
    timer = Timer(frequency=freq, callback=lambda: print(camera.camera_loop()))
    timer.start()

def start_all_cameras():
    serial_devices = get_serial_devs()
    for device in serial_devices:
        process = Process(target=lambda: camera_wrap(serial_number=device))
        process.start()

if __name__ == "__main__":
    start_all_cameras()

import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLO
import rospy
from geometry_msgs.msg import Point

NODE_NAME  = 'camera'

class CameraNode():
    def __init__(self):
        TOPIC_NAME = 'camera_info'
        self.point_publisher = rospy.Publisher(TOPIC_NAME, Point, queue_size=10)
    def publish_message(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        self.point_publisher.publish(point)

def camera_init():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    return pipeline

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

def find_closest_color(rgb_pixel, COLOR_DICT):
    min_distance = float('inf')
    closest_color = ""
    for color_name, color_rgb in COLOR_DICT.items():
        # Calculate the Euclidean distance
        distance = sum((c1 - c2) ** 2 for c1, c2 in zip(rgb_pixel, color_rgb)) ** 0.5
        if distance < min_distance:
            min_distance = distance
            closest_color = color_name
    return closest_color


def get_frames(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    return frames, color_frame, depth_frame

def align_frames(pipeline, frames):
    align = rs.align(rs.stream.color)
    aligned_frames = align.process(frames)
    return aligned_frames.get_depth_frame(), aligned_frames.get_color_frame()

def detect_objects(model, img):
    return model(img, stream=True)

def process_detection(boxes, img, depth_frame):
    for box in boxes:
        bbox_coords = box.xyxy.squeeze().cpu().numpy()
        draw_bounding_box(img, bbox_coords)
        calculate_and_display_depth(img, bbox_coords, depth_frame)

def draw_bounding_box(img, bbox_coords):
    x1, y1, x2, y2 = map(int, bbox_coords)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

def calculate_and_display_depth(img, bbox_coords, depth_frame):
    x1, y1, x2, y2 = map(int, bbox_coords)
    mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
    depth = depth_frame.get_distance(mid_x, mid_y)
    depth_text = f"Depth: {depth:.2f}m"
    cv2.putText(img, depth_text, (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def camera_loop(event):
    QUIT = 'q'
    WINDOW_NAME = 'RealSense Camera'

    frames, color_frame, depth_frames = get_frames(pipeline)
    if not color_frame:
        return
    aligned_depth_frame, aligned_color_frame = align_frames(pipeline, frames)
    img = np.asanyarray(aligned_color_frame.get_data())
    results = detect_objects(model, img)
    
    for result in results:
        process_detection(result.boxes, img, aligned_depth_frame)
    
    cv2.imshow(WINDOW_NAME, img)
    if cv2.waitKey(1) == ord(QUIT):
        rospy.signal_shutdown()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    MODEL_NAME = 'best.pt'
    rospy.init_node(NODE_NAME, anonymous=True)
    cameraNode = CameraNode()
    pipeline = camera_init()
    model = YOLO(MODEL_NAME)
    timer = rospy.Timer(rospy.Duration(.01), callback=camera_loop)
    rospy.spin()
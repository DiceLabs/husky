#!/usr/bin/env python3

import os
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

def save_image(msg,bridge,model,image_list):

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # You can perform further processing on the image here if needed
        timestamp = msg.header.stamp.to_nsec()  # Get timestamp in nanoseconds
        #filename = os.path.join(output_directory, f"image_{timestamp}.jpg")
        #cv2.imwrite(filename, cv_image)
        result = model(cv_image, verbose=True)[0]
        annotated_frame = result.plot()
        image_list.append(annotated_frame)
        #cv2.imshow("YOLOv8 Inference", annotated_frame)
        #cv2.waitKey(1)

    except Exception as e:
        print(f"Error converting and saving image: {str(e)}")

def create_video_from_images(images, output_video_file, fps):
    if not images:
        return

    height, width, _ = images[0].shape
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out = cv2.VideoWriter(output_video_file, fourcc, fps, (width, height))

    for img in images:
        out.write(img)

    out.release()
def extract_images_from_bag(bag_file, output_directory):
    bridge = CvBridge()
    model = YOLO('/home/philip/YOLO_models_in_pt_config/yolov8l-seg.pt')
    image_list=[]
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, timestamp in bag.read_messages(topics=["/realsense/color/image_raw"]):
            save_image(msg, bridge,model,image_list)

    return image_list

if __name__ == "__main__":
    # Replace the following paths with your .bag file and output directory paths
    bag_file = "/home/philip/test_bag.bag"
    output_directory = "/home/philip/test_videos/"
    output_video_file = "/home/philip/test_videos/video.avi"

    images = extract_images_from_bag(bag_file, output_directory)

    # Set the frames per second for the output video
    fps = 30



    if images:
        create_video_from_images(images, output_video_file, fps)
        print("Video creation complete.")
    else:
        print("No images found in the bag file.")



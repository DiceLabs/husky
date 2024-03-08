import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLO


def find_closest_color(rgb_pixel, color_dict):
    min_distance = float('inf')
    closest_color = ""
    for color_name, color_rgb in color_dict.items():
        # Calculate the Euclidean distance
        distance = sum((c1 - c2) ** 2 for c1, c2 in zip(rgb_pixel, color_rgb)) ** 0.5
        if distance < min_distance:
            min_distance = distance
            closest_color = color_name
    return closest_color

# Common color dictionary
color_dict = {
    "Red": (255, 0, 0),
    "Green": (0, 255, 0),
    "Blue": (0, 0, 255),
    "Yellow": (255, 255, 0),
    "Cyan": (0, 255, 255),
    "Magenta": (255, 0, 255),
    "White": (255, 255, 255),
    "Black": (0, 0, 0),
    # Add more colors as needed
}



# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


pipeline.start(config)



model = YOLO('best.pt')
              

try:
    while True:
        # Capture frame from RealSense camera
        frames = pipeline.wait_for_frames(11000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Create an align object
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        intrinsics = depth_profile.get_intrinsics()

        # Access focal length
        fx = intrinsics.fx  # Focal length x
        fy = intrinsics.fy  # Focal length y

        print("Focal Length (fx):", fx)
        print("Focal Length (fy):", fy)

        if not color_frame:
            continue
        align = rs.align(rs.stream.color)

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        # Convert to numpy array
        img = np.asanyarray(color_frame.get_data())

        # Perform object detection
        results = model(img, stream=True)
        img_width = img.shape[1]  # Get the width of the image
        img_height = img.shape[0]  # Get the height of the image

        # Calculate the horizontal center of the image
        image_center_x = img_width // 2
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            masks = result.masks  # Masks object for segmentation masks outputs
            keypoints = result.keypoints  # Keypoints object for pose outputs
            probs = result.probs  # Probs object for classification outputs

            # Process each box in the detection result
            for box in boxes:
                # Extract bounding box coordinates
                bbox_coords = box.xyxy.squeeze().cpu().numpy()  
                x1, y1, x2, y2 = bbox_coords
                # Convert to integers if necessary
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                #Image Center   
                image_center_x = img_width // 2
                image_center_y = img_height // 2
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2
                depth = depth_frame.get_distance(mid_x, mid_y)
                # Display bounding box and other info on the image as needed
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                # Distance from middle of the image i.e. center of the camera
                dx = mid_x - image_center_x
                dy = mid_y - image_center_y
                #real_dx and real_dy should give real world dimension offsets, we can probably use threshold to dynamicallty adjust the arms
                real_dx = (dx* 0.7 * depth) / fx
                real_dy = (dy* 0.7 * depth) / fy
                distance_text = f"Real DX: {real_dx:.2f}m, Real DY: {real_dy:.2f}m"

                # Create a new image (black background) for displaying the text
                # This can be adjusted based on how much text you want to display or the preferred size
                info_img = np.zeros((200, 600, 3), dtype=np.uint8)

                # Display the distance text on the new image
                cv2.putText(info_img, distance_text, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Display the new window with real-world distance information
                cv2.imshow('RealSense Distance Info', info_img)
                print("Real World Dimension Offsets -->",  0.7 * depth / fy,  0.7 * depth / fy)
                #Calculate the Centeredness? of the identified object
                bbox_center_x = (x1 + x2) // 2
                if abs(bbox_center_x - image_center_x) <= 20:
                    bbox_color = (0, 255, 0)  # Green color for centered bounding box
                else:
                    bbox_color = (0, 0, 255)
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 3)

                depth_text = f"Depth: {depth:.2f}m"
                cv2.putText(img, depth_text, (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Calculate real-world dimensions 
                width_px = x2 - x1
                height_px = y2 - y1

                # Calculate real-world dimensions
                real_width_m = (width_px* 0.7 * depth) / fx
                real_height_m = (height_px* 0.7 * depth) / fy
                
                 # Format the dimension text
                dimension_text = f"Width: {real_width_m:.2f}m, Height: {real_height_m:.2f}m"

                # Define text position (just above the bounding box)
                text_position = (x1, y2 - 20)

                # Display the dimensions on the image
                cv2.putText(img, dimension_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


                # Display depth, RGB, and color name
                rgb_pixel = img[mid_y, mid_x, :]
                color_name = find_closest_color(rgb_pixel, color_dict)
                color_text = f"Color: {color_name}"
                cv2.putText(img, depth_text + " " + color_text, (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                #Confidence
                confidence = math.ceil((box.conf[0]*100))/100
                print("Confidence --->", confidence)

                # Display class name
                #cls = int(box.cls[0])
               # print("Class name -->", class_names[cls])

                # Put text on image
                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2
                #cv2.putText(img, class_names[cls], org, font, fontScale, color, thickness)

        # Display image
        cv2.imshow('RealSense Camera', img)

        # Exit on 'q' press
        if cv2.waitKey(1) == ord('q'):
            break
finally:
    # Stop RealSense camera
    pipeline.stop()
    cv2.destroyAllWindows()

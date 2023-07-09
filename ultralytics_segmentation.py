#!/usr/bin/python3
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from ultralytics import YOLO  #pip3 install ultralytics
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float64MultiArray
import cv2
from sensor_msgs.msg import CameraInfo
import cProfile
import math

print("HELL")

def profileit(name):
    def inner(func):
        def wrapper(*args, **kwargs):
            prof = cProfile.Profile()
            retval = prof.runcall(func, *args, **kwargs)
            # Note use of name from outer scope
            prof.dump_stats(name)
            return retval
        return wrapper
    return inner

class camera_detect:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO('yolov8s-seg.pt')
        self.cv_image = None
        self.result = None
        self.class_names = None
        self.boxes = None
        self.info = None
        self.distance = None
        self.c1_time_stamp = None
        self.c2_time_stamp = None
        self.Center_x = None # x coordinate of bounding box center
        self.Center_y = None # y coordinate of bounding box center
        self.color_image_height = None #Height of opencv color image
        self.color_image_width = None #Width of opencv color image
        self.ll=[]
        self.obj_found = False
        self.image=None
        self.camera_intrinsics=None
        self.depth_img=None
        #self.R = np.array([[0, 1, 0],
         #             [0, 0, -1],
         #             [-1, 0, 0]])
        self.camera_matrix = None

        self.the_points = Float64MultiArray()

        self.deltas = Float64MultiArray()
        self.pub1 = rospy.Publisher("/the_points", Float64MultiArray, queue_size=1, latch=True)

        self.pub2 = rospy.Publisher("/deltas", Float64MultiArray, queue_size=1, latch=True)

    """subscriber to Camerainfo messages"""
    def callback0(self,msg):
        self.camera_intrinsics = msg

        """Reshape K matrix"""
        self.camera_matrix = np.array(self.camera_intrinsics.K).reshape(3, 3)


    def callback1(self,msg):
        self.image = msg


    def callback2(self, msg):
        self.depth_img = msg

    def get_optical_frame_coordinate(self):
        try:
            self.c1_time_stamp = self.image.header.stamp
            self.c2_time_stamp = self.depth_img.header.stamp
            time1 = rospy.Time(self.c1_time_stamp.secs, self.c1_time_stamp.nsecs)
            time2 = rospy.Time(self.c2_time_stamp.secs, self.c2_time_stamp.nsecs)
            time_diff = (time2 - time1).to_sec()
        except:
            pass
        try:
            if abs(time_diff)<0.9:
                print("TIME_DIFF: ",time_diff)
                self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')    #'rgb8')
                self.result = self.model(self.cv_image, verbose=True)[0]
                self.boxes = self.result.boxes.data.cpu().numpy()
                self.class_names = self.result.names
                #print(self.result.cpu().masks.xy)

                if len(self.boxes) > 0:
                    try:
                        for i in range(len(self.boxes)):
                            class_index = int(self.boxes[i][5])
                            if str(self.class_names[class_index]) == "fire hydrant":  # or str(self.class_names[class_index]) == "person":
                                # convert to opencv depth image
                                depth_image = self.bridge.imgmsg_to_cv2(self.depth_img,
                                                                        desired_encoding='passthrough')  # '16UC1')
                                # print([self.cv_image.shape[0],self.cv_image.shape[1],depth_image.shape[0],depth_image.shape[1]])

                                # Apply scaling factor parameters that relate pixel locations of color image and depth image
                                scale_factor_col = depth_image.shape[1] / self.cv_image.shape[1]
                                scale_factor_row = depth_image.shape[0] / self.cv_image.shape[0]
                                self.info = self.result[i].cpu().masks.xy[0]

                                #Get center of bounding box
                                self.Center_x,self.Center_y = np.mean(np.array(self.info),axis=0)
                                delta_x = (int(self.Center_x) - (self.cv_image.shape[1] // 2))  # self.bottle_center_x
                                delta_y = (int(self.Center_y) - (self.cv_image.shape[0] // 2))

                                col = int(self.Center_x * scale_factor_col)
                                row = int(self.Center_y  * scale_factor_row)

                                #Get Z_depth in meters of pixel in depth image
                                z_depth = (depth_image[row][col]/1000)
                                self.deltas.data = [delta_x, delta_y, z_depth]
                                self.pub2.publish(self.deltas)
                                #z_depth*=0.30


                                x_ndc = (col - self.camera_intrinsics.K[2]) / self.camera_intrinsics.K[0]
                                y_ndc = (row - self.camera_intrinsics.K[5]) / self.camera_intrinsics.K[4]

                                #Scale the normalized device coordinates to obtain the 3D coordinates in camera coordinate space
                                x_cam = x_ndc * (z_depth) / 1.0
                                y_cam = y_ndc * (z_depth) / 1.0
                                #rad = math.atan2(x_cam, z_depth) #math.radians(math.degrees(math.atan2(x_cam, z_depth) + 360) % 360)
                                #PAY ATTENTION TO THE Z_DEPTH, THESE POSITION VALUES ARE IN METERS. IF Z_DEPTH VALUE DOES NOT LOOK CORRECT THEN IT'S 
                                # A RESULT OF INACCURATE READINGS.
                                print("CLASS_NAME: ", str(self.class_names[class_index]), " ,3D POINT in depth_camera_optical_frame: ",[x_cam ,y_cam,z_depth],"\n")
                                #print("RADS: ",  rad)
                                self.the_points.data= [x_cam ,y_cam,z_depth]
                                self.pub1.publish(self.the_points)

                    except IndexError:
                        pass
                        print("INDEX ERROR \n")


        except UnboundLocalError:
            pass



#@profileit("/home/philip/Desktop/output.txt")
def main():

        rospy.init_node("perceive_objects")
        tester = camera_detect()

        rospy.Subscriber("/realsense/depth/camera_info", CameraInfo, tester.callback0, queue_size=100)
        rospy.Subscriber("/realsense/color/image_raw", Image, tester.callback1, queue_size=1, buff_size=2_000_000)
        rospy.Subscriber("/realsense/depth/image_rect_raw", Image, tester.callback2, queue_size=1, buff_size=2_000_000)

        while not rospy.is_shutdown():
            tester.get_optical_frame_coordinate()


if __name__ == "__main__":
    main()

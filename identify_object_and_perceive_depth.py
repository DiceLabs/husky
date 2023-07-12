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

#Helps calculate centroids of polygons created by instance segmentation
from shapely.geometry import Point, Polygon

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
        self.model = YOLO('/home/philip/YOLO_models_in_pt_config/yolov8l-seg.pt')
        self.cv_image = None
        self.result = None
        self.class_names = None
        self.boxes = None
        self.polygon = None
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
                                scale_factor_col = depth_image.shape[1] / self.cv_image.shape[1] # scale factor for columns
                                scale_factor_row = depth_image.shape[0] / self.cv_image.shape[0] # scale factor for rows
                                self.polygon = self.result[i].cpu().masks.xy[0]

                                instance_segment = Polygon(np.array(self.polygon))
                                #print("POLYGON: ",np.array(self.polygon))
                                self.Center_col = instance_segment.centroid.x
                                self.Center_row = instance_segment.centroid.y
                                #delta_col = (int(self.Center_col) - (self.cv_image.shape[1] // 2))  # self.bottle_center_x
                                #delta_row = (int(self.Center_row) - (self.cv_image.shape[0] // 2))

                                col = int(self.Center_col * scale_factor_col)
                                row = int(self.Center_row  * scale_factor_row)

                                #FIND PIXELS IN POLYGON ATTEMPT 1
                                #num_columns = self.cv_image.shape[1] #int(self.boxes[i][2]) -  int(self.boxes[i][0])
                                #num_rows = self.cv_image.shape[0] #int(self.boxes[i][3]) -  int(self.boxes[i][1])
                                #print(num_columns,", ",num_rows)
                                """x_coords = np.linspace(0, num_columns - 1, num_columns)
                                y_coords = np.linspace(0, num_rows - 1, num_rows)
                                x_grid, y_grid = np.meshgrid(x_coords, y_coords)
                                points = np.column_stack((x_grid.flatten(), y_grid.flatten()))
                                points_inside = [point for point in points if instance_segment.contains(Point(point))]"""

                                # FIND PIXELS IN POLYGON ATTEMPT 2
                                # Create an empty black image as the binary mask
                                mask = np.zeros((self.cv_image.shape), dtype=np.uint8) #might have to change dtype for bigger images

                                # Convert polygon vertices to a numpy array of shape (num_points, 1, 2)
                                polygon_pts = np.array(self.polygon, np.int32)
                                polygon_pts = polygon_pts.reshape((-1, 1, 2))

                                # Draw the filled polygon on the mask
                                cv2.fillPoly(mask, [polygon_pts], color=255)
                                #cv2.imshow("Mask", mask)
                                #cv2.waitKey(2)
                                # Get the positions (row, column) where the mask is 255 (inside the polygon)
                                scaled_positions = np.array(np.argwhere(mask == 255)) #* np.array([scale_factor_row,scale_factor_col]))
                                #print(scaled_positions)
                                row_indices =  (scaled_positions[:, 0] * scale_factor_row).round().astype(int)
                                column_indices = (scaled_positions[:, 1] * scale_factor_col).round().astype(int)
                                depth_image = np.array(depth_image)
                                #depths = depth_image[row_indices, column_indices]
                                #for element in depths:
                                #    print(element)


                                #print(row_indices)

                                #Get Z_depth in meters of pixel in depth image
                                z_depth = np.nanmean(depth_image[row_indices,column_indices])/1000   #depth_image[row][col]#/1000
                                #print("NAN: ",np.argwhere(np.isnan(depth_image[row_indices,column_indices])))
                                #print(z_depth)
                                #self.deltas.data = [delta_col, delta_row, z_depth]
                                #self.pub2.publish(self.deltas)
                                #z_depth*=0.30


                                x_ndc = (col - self.camera_intrinsics.K[2]) / self.camera_intrinsics.K[0]
                                y_ndc = (row - self.camera_intrinsics.K[5]) / self.camera_intrinsics.K[4]

                                #Scale the normalized device coordinates to obtain the 3D coordinates in camera coordinate space
                                x_cam = x_ndc * (z_depth) / 1.0
                                y_cam = y_ndc * (z_depth) / 1.0


                                print("CLASS_NAME: ", str(self.class_names[class_index]), " ,3D POINT in depth_camera_optical_frame: ",[x_cam ,y_cam,z_depth],"\n")

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

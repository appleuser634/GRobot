import pyrealsense2 as rs
import numpy as np
import cv2

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,Point

rospy.init_node("marker_pub")

pub = rospy.Publisher("arrow_pub", Marker, queue_size = 10)
rate = rospy.Rate(25)


marker_data = Marker()
marker_data.points = []

def publish_marker_position(x,y):
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD

    marker_data.pose.position.x = x
    marker_data.pose.position.y = y
    marker_data.pose.position.z = 0.0

    #add_point = Point()
    #add_point.x = x
    #add_point.y = 1.0
    #add_point.z = 1.0

    #marker_data.points.append(add_point)

    marker_data.pose.orientation.x=0.0
    marker_data.pose.orientation.y=0.0
    marker_data.pose.orientation.z=1.0
    marker_data.pose.orientation.w=0.0

    marker_data.color.r = 1.0
    marker_data.color.g = 1.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 5
    marker_data.scale.y = 5
    marker_data.scale.z = 0.1

    marker_data.lifetime = rospy.Duration()

    marker_data.type = 1

    pub.publish(marker_data)

    rate.sleep()

dictionary_name = cv2.aruco.DICT_4X4_50
dictionary = cv2.aruco.getPredefinedDictionary(dictionary_name)

pipeline =  rs.pipeline()

config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while not rospy.is_shutdown():
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))


        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, dictionary)
        if len(corners) > 0:
        # flatten the ArUco IDs list
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                
                if markerID != 10:
                    continue

                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(color_image, (cX, cY), 4, (0, 0, 255), -1)
                #color_image = cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                #print("CX:",cX,"CY:",cY)
                
                publish_marker_position(cX/10, cY/10)
                break

        
        #print("Shape:",color_image.shape[:2])

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        cv2.imshow('show image!',color_image)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()


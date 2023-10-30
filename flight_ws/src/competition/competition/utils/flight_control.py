import cv2
import numpy as np
from matplotlib import pyplot as plt
from pupil_apriltags import Detector
import time

############ params ############
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
# pin hole model
cx = width / 2.0
cy = height / 3.0
fx = 600
fy = 600
intrinsics = ([fx, fy, cx, cy])
K = np.array([[fx, 0.0, cx],
              [0.0, fy, cy],
              [0.0, 0.0, 1.0]])
# tag size (m)
tag_size = 0.165
###############################
target_dist = 0.0 # desired distance from tag
# remember left-right and up-down error is larger
# than forward-back error since the first two are
# in pixels and not meters!
P_z = 0.0 #7 #11
 # forward-back gain
P_x = 0.025 #0.12 # left-right gain
P_y = 0.01 # 0.27 # up-down gain
target_tag = "0" # tag to target
target_tags = ["0", "1", "2", "3", "4", "5", "6", "7"]
tag_ind = 0
###############################
# initialize detector
at_detector = Detector(families='tag36h11',
                   nthreads=1,
                   quad_decimate=1.0,
                   quad_sigma=0.0,
                   refine_edges=1,
                   decode_sharpening=0.25,
                   debug=0)
fly = True
# feature tracking 
# Parameters for lucas kanade optical flow
lk_params = dict( winSize = (45, 45),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.1) )
u1 = np.zeros((1, 1, 2))
u2 = np.zeros((1, 1, 2))
old_gray = None
new_gray = None
point_below = None
flow_point = None
tracking_flow = False
###############################
cross_threshold = 55
time_last_crossed = 0
### Scanning params
ccw = True
counter = 0
###############################

# piece 2
def check_crossed_target(dist):
        global time_last_crossed
        # wait this long between detections
        # otherwise you will count multiple crossing when you only crossed once
        cutoff_time = 7

        # get current drone altitude
        if dist < cross_threshold:
                # altitude is below threshold, so we detected an obstacle
                # check cutoff time
                if time.time() - time_last_crossed > cutoff_time:
                        # new obstacle detected, reset cutoff time
                        time_last_crossed = time.time()
                        return True
        return False

def get_point_below_tag(T_camera_tag, meters_below, K):


    """
    given a 4x4 transform FROM the tag TO the camera,
    compute the 3D location of a point meters_below the tag
    and return the 2D location of that point in the image frame.
    """
    # get inverse transform
    T_tag_camera = np.linalg.inv(T_camera_tag)
    # find 3D location of point below the tag
    T_tag_camera[1,3] -= meters_below
    # convert back to being wrt the camera frame
    T_camera_tag = np.linalg.inv(T_tag_camera)
    R_body_camera = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    print(R_body_camera.shape)
    print(T_camera_tag.shape)
    #T_body_tag = R_body_camera @ T_camera_tag[0:3, 3:]
    ###### TODO fill in here ##########
    # compute the 2D location of the target point in the image plane
    # hint: T_camera_tag[0:3,3:] is the 3D point of the target wrt the camera
    # hint: don't forget to return a 2D point and not a 3D point
    # px = T_camera_tag[0:3,3:]/T_camera_tag[0:3,3:][2]
    x_img = K@T_camera_tag[0:3,3:]
    #x_img = K@T_body_tag
    x_img = x_img/x_img[2]
    x_img = x_img[0:2, :]
    px = int(x_img[0][0])
    py = int(x_img[1][0])
    print(px)
    print("matrix transform worked hopefully")
    return (px, py)
    
    ##################################

def detect_tags(img, target_point_dist = None, visualize = False):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(img_gray, estimate_tag_pose=True,
                              camera_params=intrinsics, tag_size=tag_size)
    print("number of tags detected:", len(tags))
    tag_info = {}
    # loop over the AprilTag detection results
    global point_below
    global flow_point
    global prev_flow_point
    for tag in tags:
        if str(tag.tag_id) != target_tag:
            continue
        target = None
    	# extract the bounding box (x, y)-coordinates for the AprilTag
    	# and convert each of the (x, y)-coordinate pairs to integers
        print("tag id")
        print(tag.tag_id)
        if visualize: 
            (ptA, ptB, ptC, ptD) = tag.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(img, ptA, ptB, (0, 255, 0), 2)
            cv2.line(img, ptB, ptC, (0, 255, 0), 2)
            cv2.line(img, ptC, ptD, (0, 255, 0), 2)
            cv2.line(img, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = tag.tag_family.decode("utf-8")
            cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        ##### Detect point set distance below tag #####
        # construct 4x4 transform matrix of the tags location wrt the camera frame
        if target_point_dist != None:
            T_camera_tag = np.zeros((4,4))
            T_camera_tag[3,3] = 1.0
            T_camera_tag[0:3,3:] = tag.pose_t
            T_camera_tag[0:3,0:3] = tag.pose_R
            point_below = get_point_below_tag(T_camera_tag, meters_below=0.45, K=K)
            if visualize:
                cv2.line(img, point_below, (cX, cY), (255, 0, 0) , 3)
            target = {'position': T_camera_tag, 'pixel_coords': point_below}
            print("got target id for tag")
        tag_info[str(tag.tag_id)] = {'tag': tag, 'target': target}
    if point_below != None:
        u2 = get_flow_point(point_below, img)
        point_below = u2
        flow_point = u2
    if visualize:
        if flow_point != None:
            cv2.circle(img, flow_point, 10, (0, 255, 0), 2)
        cv2.imshow("drone cam", img)
        key = cv2.waitKey(50)
    return tag_info

def get_flow_point(point_below, img):
    global new_gray
    global old_gray
    old_gray = new_gray
    print("old gray")
    new_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if old_gray is not None: 
        u1 = np.reshape(np.array([point_below[0], point_below[1]],dtype="float32"), (1,1,2))
        u2, _, _ = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, u1, None, **lk_params)
        print("firstU2")
        print(u2)
        px = int(u2[0][0][0])
        py = int(u2[0][0][1])
        return (px, py)

def get_z_control(target, observed, gain):
    return int(gain * (observed - target))

def get_left_right_control(target, observed, gain):
    return int(gain * (observed - target))

def get_up_down_control(target, observed, gain):
    return int(gain * -(observed - target))

def fly_drone(frame, dist):
    global tracking_flow
    img = cv2.resize(frame, (width, height))
    cv2.imshow("drone cam", img)
    cv2.waitKey(1)
    # track tagss
    tag_data = detect_tags(img, target_point_dist = target_dist, visualize = True)
    # extract tag location and control drone
    target_tag = target_tags[tag_ind]
    print(target_tag)
    if target_tag in tag_data and not tracking_flow:
        print('inside')
        z_dist=2
        #z_dist = tag_data[target_tag]['tag'].pose_t[2]
        # get velocity components
        x_pos = tag_data[target_tag]["target"]["pixel_coords"][0]
        y_pos = tag_data[target_tag]["target"]["pixel_coords"][1]
        left_right_vel = get_left_right_control(cx, x_pos, gain = P_x)
        if abs(x_pos - cx) > 40: 
            forward_vel = forward_vel = get_z_control(target_dist, z_dist, gain=P_z*0.6)
            up_down_vel = get_up_down_control(cy, y_pos, gain = P_y*0.5)
        else:
            forward_vel = get_z_control(target_dist, z_dist, gain=P_z*1.8)
            up_down_vel = get_up_down_control(cy, y_pos, gain = P_y)
        # left_right_vel = get_left_right_control( 
        #     cx, tag_data[target_tag]['tag'].center[0], gain=P_x)
        # up_down_vel = get_up_down_control(
        #     cy, tag_data[target_tag]['tag'].center[1] - 15, gain=P_y)
        yaw_velocity = 0
        # send control action to drone
        if fly: 
            return left_right_vel, forward_vel, up_down_vel, yaw_velocity
        else: 
            print(left_right_vel, forward_vel, up_down_vel, yaw_velocity)
            return 0, 0, 0, 0
    else:
        print("WE SHOULD BE HERE DOG")
        # if we don't see the tag, do the safe thing and stop the drone
        if flow_point != None:
            # if abs(flow_point[0] - prev_flow_point[0]) > 10 and prev_flow_point != None: # to prevent the flow point from changing drastically
            #     flow_point = prev_flow_point # counters drift
            # else:
            #     prev_flow_point = flow_point
            print("flow point active")
            x = flow_point[0]
            y = flow_point[1]
            z_dist = 0.5
            forward_vel = get_z_control(target_dist, z_dist, gain=P_z)
            left_right_vel = get_left_right_control(cx, x, gain=P_x)
            up_down_vel = get_up_down_control(cy, y, gain=P_y)
            yaw_velocity = 0
            print(dist)
            if check_crossed_target(dist):
                tracking_flow = False
                #tracking_flow = True # tracking the flow point 
            if fly: 
                return left_right_vel, forward_vel, up_down_vel, yaw_velocity
            else:
                print(left_right_vel, forward_vel, up_down_vel, yaw_velocity)
                return 0, 0, 0, 0
        else: #
            print("going up!")
            if fly: 
                return 0, 0, 0, 0
            else:
                print(0, 0, 1, 0)
                return 0, 0, 0, 0
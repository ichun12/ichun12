from multiprocessing import Process
import time
import socket
from tokenize import Pointfloat
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import math

angle = 0

def cam() :
    global corners, angle
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
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
    pipeline.start(config)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            img_point = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            ret, imthres = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY_INV)
            
            # goodFeaturesToTrack()
            K = 4
            corners = cv2.goodFeaturesToTrack(gray_img, maxCorners=K, qualityLevel=0.05, minDistance = 10)
            corners = corners.reshape(-1,2)
            global line_len
            line_len = list()

            for x,y in corners:
                cv2.circle(color_image,(x, y),4,(0,0,255),-1)

            # get the shortest line's length
            for i in range(6):
                if i == 3:
                    line_len.append(math.sqrt((corners[i, 0] - corners[i-3,0])**2 + (corners[i,1] - corners[i-3,1])**2))
                # point1 - point3
                elif i == 4 : 
                    line_len.append(math.sqrt((corners[i-3, 0] - corners[i-1, 0])**2 + (corners[i-3,1] - corners[i-1,1])**2))
                #point0 - point2
                elif i == 5:
                    line_len.append(math.sqrt((corners[i-5,0] - corners[i-3,0])**2 + (corners[i-5,1] - corners[i-3,1])**2))
                    print("fixed")
                else:
                    line_len.append(math.sqrt((corners[i, 0] - corners[i+1,0])**2 + (corners[i,1] - corners[i+1,1])**2))

            # get square's middle point
            mid_point_x = 0
            mid_point_y = 0
            mid_point = [0,0]

            for i in range(4):
                mid_point_x += corners[i,0]
                mid_point_y += corners[i,1]
            
            px_m = mid_point_x/4
            py_m = mid_point_y/4
            
            mid_point = [px_m, py_m]

            # make circle on the middle point
            cv2.circle(color_image,(int(px_m), int(py_m)),4,(255,0,0),-1)

            for i in range(6):
                if min(line_len) == line_len[i]:                    
                    if i == 3:
                        A_px = (corners[i,0] + corners[i-3,0])/2
                        A_py = (corners[i,1] + corners[i-3,1])/2
                        cv2.line(color_image, (corners[i,0], corners[i,1]), (corners[i-3,0],corners[i-3,1]),(255,0,0),3)
                    elif i == 4:
                        A_px = (corners[i-3,0] + corners[i-1,0])/2
                        A_py = (corners[i-3,1] + corners[i-1,1])/2
                        cv2.line(color_image, (corners[i-3,0], corners[i-3,1]), (corners[i-1,0],corners[i-1,1]),(255,0,0),3)
                    elif i == 5:
                        A_px = (corners[i-5,0] + corners[i-3,0])/2
                        A_px = (corners[i-5,1] + corners[i-3,1])/2
                        cv2.line(color_image, (corners[i-5,0], corners[i-5,1]), (corners[i-3,0],corners[i-3,1]),(255,0,0),3)
                    else:
                        
                        A_px = (corners[i,0] + corners[i+1,0])/2
                        A_py = (corners[i,1] + corners[i+1,1])/2
                        cv2.line(color_image, (corners[i,0], corners[i,1]), (corners[i+1,0],corners[i+1,1]),(255,0,0),3)
        
            A_point = [A_px, A_py]
            
            cv2.circle(color_image,(int(A_px), int(A_py)), 4, (0,255,0),1)
            
            # get angle
            base = 0
            height = 0
            rad = 0.0
            angle = 0.0

            base = max(A_point[0], mid_point[0]) - min(A_point[0], mid_point[0])
            height = max(A_point[1], mid_point[1]) - min(A_point[1], mid_point[1])
            rad = math.atan2(height, base)
            angle = (rad*180) / math.pi
            
            # 1
            if (mid_point[0] > A_px) and (mid_point[1] > A_py):
                angle = 180 - angle
            elif (mid_point[0] <= A_px) and (mid_point[1] <= A_py):
                angle = angle * (-1)
            elif (mid_point[0] > A_px) and (mid_point[1] <= A_py):
                angle = -1 * (180 - angle) 
            else:
                angle = angle

            print("mid_point : ", mid_point)
            print("A_point : ", A_point)
            print(int(angle), " degree")
            print('\n')

            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('color', color_image)
            cv2.waitKey(1)

    finally:
        # Stop streaming
        pipeline.stop()

def client() :
    global corners, line_len, angle

    HOST='192.168.1.196'
    PORT=5005

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))

    while True:
        
        client_socket.sendall(str(angle).encode('utf-8'))
        time.sleep(0.5)
        # client_socket.sendall(str(py).encode('utf-8'))
        # time.sleep(0.5)
        #data1 = client_socket.recv(1024)
        #print('Received1', repr(data1.decode()))

t_cam = threading.Thread(target=cam)
t_client = threading.Thread(target=client)

t_cam.start()
t_client.start()

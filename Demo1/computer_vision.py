"""
Authors: John Capper
Class: EENG350
Assignment: Computer Vision Script for Mini Project

"""

from smbus2 import SMBus
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np


camera = None


def camera_init(res=None, iso=400):
    # Set global camera variable
    global camera
    camera = PiCamera(resolution=res)
    camera.iso = iso
    
    time.sleep(0.5)     # Wait for half-second
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'

    # Calibrate camera by allowing auto awb to get gain then turn awb_mode to off
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g


def capture_img(save=False, show=True, fps=0):
    if save:
        fileName = input("File Name:")
        
    rawCapture = PiRGBArray(camera)
    
    # Capture Image
    try:
        camera.capture(rawCapture, format="bgr")
        img = rawCapture.array
#        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    except Exception as e:
        print(e)
        print("Failed to capture")
    
    
    # Show Image
    if show:
        try:
            cv2.imshow("Captured Image", img)
            cv2.waitKey(fps)
        except:
            print('Failed to show image')
    
    # Save Image
    if save:
#        print("Savicamera.shutter_speed = camera.exposure_speedng image "+fileName)
        try:
            cv2.imwrite(fileName, img)
        except:
            print("Couldn't save image")
            pass    
    
    return img



def centroid_from_corners(marker_corners):
    marker_corners = marker_corners[0, :]
    top_left = marker_corners[0]
    bottom_right = marker_corners[2]

    # Get marker center
    cx = top_left[0] + (bottom_right[0]-top_left[0]) // 2
    cy = top_left[1] + (bottom_right[1]-top_left[1]) // 2
    
    return cx, cy
    

def quadrant_from_centroid(cx, cy, img_cx, img_cy):
    
    # Get marker center relative to image center
    relative_x = cx - img_cx
    relative_y = cy - img_cy

    # Find quadrant
    if relative_x >= 0 and relative_y <= 0:
        quadrant = 1
    elif relative_x < 0 and relative_y <= 0:
        quadrant = 2
    elif relative_x < 0 and relative_y > 0:
        quadrant = 3
    else:
        quadrant = 4

    return quadrant
    


def angle_from_centroid(cx, img_cx, horizontal_fov):
    
    return ((img_cx-cx)/img_cx) * (horizontal_fov / 2)


def detect_aruco(bgr_img=None, get_all=False, show_capture=False, video_debug=False):
    if bgr_img is None:
        bgr_img = capture_img(show=show_capture or video_debug, fps=10 if video_debug else 0)   # Take Picture, will continuously show capture if debug is on

    gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

    parameters = cv2.aruco.DetectorParameters_create()  # Create default detector parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)   # Define correct dictionary

    # Do aruco detection
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_img,
                                                              aruco_dict,
                                                              parameters=parameters)


    # If no markers detected
    if ids is None:
        print("No markers found")
        return None
    
    # Get image center
    img_height, img_width = gray_img.shape
    img_cy = img_height // 2
    img_cx = img_width // 2
    
    horizontal_fov = 60.3
    
    results = []
    for marker_id, marker_corners in zip(ids, corners):   
 
        cx, cy = centroid_from_corners(marker_corners)  # Find centroid of marker
        
        stats = {
            'id': marker_id[0],
            'quadrant': quadrant_from_centroid(cx, cy, img_cx, img_cy),
            'angle': angle_from_centroid(cx, img_cx, horizontal_fov)
        }
        if not get_all:
            return stats
        
        results.append(stats)
    
    return results



if __name__ == '__main__':
    camera_init()
    
    while True:
        results = detect_aruco()
        print(results)
    
    camera.close()
    


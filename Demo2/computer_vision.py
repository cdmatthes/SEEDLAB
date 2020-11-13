"""
Authors: John Capper
Class: EENG350
Assignment: Computer Vision Script for Demo 2
"""

from smbus2 import SMBus
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np

# Marker Info
MARKER_SIZE = 2.375
half_size = MARKER_SIZE/2
P_M = np.array([
    (-half_size, half_size, 0),
    (half_size, half_size, 0),
    (half_size, -half_size, 0),
    (-half_size, -half_size, 0),
]) # Model Points

# Camera Info
camera = None
HORIZONTAL_FOV = 60.3   # Frame of View of camera

# Focal Length Calculation
# Note: f=fx=fy
# x = 1135-1014 # pixels
# Z = 36 # inches
# f = (x*Z)/MARKER_SIZE

# x = 1172-997 # pixels
# Z = 25 # inches
# f = (((x*Z)/MARKER_SIZE) + f) / 2
f = 1840.0

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
    # Testing function that prints x y coordinate of click
    def get_xy(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
    
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
            window_name = "Captured Image"
            cv2.namedWindow(window_name)
            cv2.setMouseCallback(window_name, get_xy)
            cv2.imshow(window_name, img)
            cv2.waitKey(fps)
        except:
            print('Failed to show image')
    
    # Save Image
    if save:
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


def angle_from_centroid(cx, img_cx, horizontal_fov):
    dx = img_cx-cx  # Horizontal distance from origin
    return (dx/img_cx) * (horizontal_fov / 2)   # Angle will be a proportion


def calc_marker_distance(marker_corners, P_M, K):
    # Find pose of marker from corners and corresponding model corners. Uses camera matrix to find scale
    # Will return the translational vector tvec
    _, rvec, tvec = cv2.solvePnP(
        objectPoints=P_M,
        imagePoints=marker_corners,
        cameraMatrix=K,
        distCoeffs=None
    )
    t = tvec.T[0]
    
    tx = t[0]
    tz = t[2]
    d = np.sqrt(tx**2+tz**2)
    return d


def detect_aruco(bgr_img=None, get_all=False, show_capture=False, video_debug=False):
    if bgr_img is None:
        bgr_img = capture_img(show=show_capture or video_debug, fps=10 if video_debug else 0)   # Take Picture, will continuously show capture if debug is on

    gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale


    # Make Camera Matrix
    img_height, img_width = gray_img.shape
    img_cy = img_height // 2
    img_cx = img_width // 2
    K = np.array([
        (f, 0, img_cx),
        (0, f, img_cy),
        (0, 0, 1)
    ])


    # Do aruco detection
    parameters = cv2.aruco.DetectorParameters_create()  # Create default detector parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)   # Define correct dictionary
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)

    
    # If no markers detected
    if ids is None:
        print("No markers found")
        return None
    
    results = []
    for marker_id, marker_corners in zip(ids, corners):   
        cx, cy = centroid_from_corners(marker_corners)  # Find centroid of marker
        
        angle = angle_from_centroid(cx, img_cx, HORIZONTAL_FOV)
        print(angle)
        # Make a dictionary containing info about the marker including id, quadrant, and angle
        stats = {
            'id': marker_id[0],
            'angle': angle,
            'distance': calc_marker_distance(marker_corners, P_M, K)   
        }
        
        if not get_all: # Just return one marker if needed
            return stats
        
        results.append(stats)
    
    return results


# Strictly for testing, our program is ran from system_integration.py
if __name__ == '__main__':
    camera_init()
    results = detect_aruco()
    print(results)
    camera.close()
    



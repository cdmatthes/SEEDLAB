from smbus2 import SMBus
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

bus = SMBus(1)

address = 0x04

lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

camera = None

def writeString(value): #send to arduino
    bus.write_byte_data(address,0,value)
    return -1

def readNumber(): #receive info from arduino
    number = bus.read_byte_data(address, 0)
    return number


def camera_init(res=None, iso=400):
    global camera
    camera = PiCamera(resolution=res)
    camera.iso = iso
    
    time.sleep(0.5) # Wait for half-second
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g


def capture_img(save=False, show=True):
    if save:
        fileName = input("File Name:")
        
    rawCapture = PiRGBArray(camera)
    
    # Capture Image
    try:
        camera.capture(rawCapture, format="bgr")
        img = rawCapture.array
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    except Exception as e:
        print(e)
        print("Failed to capture")
    
    
    # Show Image
    if show:
        try:
            cv2.imshow("Image", img)
            cv2.waitKey(0)
        except:
            print('Failed to show image')
    
    # Save Image
    if save:
        print("Savicamera.shutter_speed = camera.exposure_speedng image "+fileName)
        try:
            cv2.imwrite(fileName, img)
        except:
            print("Couldn't save image")
            pass    
    
    return img


def convert_2_gray(img, show=True):
    g_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if show:
        cv2.imshow("Grayscale Image", g_img)
        cv2.waitKey(0)

    return g_img


def detect_aruco(bgr_img=None, get_info=True):
    if bgr_img is None:
        bgr_img = capture_img(save=False, show=False)
        

    gray_img = convert_2_gray(bgr_img, show=False)  # Convert to grayscale
#    cv2.imshow("Hey", gray_img)
#    cv2.waitKey(0)
    parameters = cv2.aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_img,
                        aruco_dict,
                        parameters=parameters)

    if ids is None:
        print("No markers found")
        return None, None
    
    ids = [n[0] for n in ids]
    if not get_info:
        return ids, None

    # If there was detection, then get angle and distance
    img_height, img_width = gray_img.shape
    img_cy = img_width // 2
    img_cx = img_height // 2
    
    quadrants = []
    for marker_corners in corners:
        marker_corners = marker_corners[0, :]
        top_left = marker_corners[3]
        bottom_right = marker_corners[1]
        cy = top_left[0] + bottom_right[0]//2
        cx = top_left[1] + bottom_right[1]//2

        relative_x = cx - img_cx
        relative_y = cy - img_cy
        
        if relative_x >= 0 and relative_y >= 0:
            quadrant = 1
        elif relative_x < 0 and relative_y >= 0:
            quadrant = 2
        elif relative_x < 0 and relative_y < 0:
            quadrant = 3
        else:
            quadrant = 4
        
        quadrants.append(quadrant)
        
    return ids, quadrants


def continuous_aruco_detection():
    while True:
        ids, quadrants  = detect_aruco()
        if ids is not None:
            print("ID:", ids)
            print("Quadrants:", quadrants)
            writeString(quadrants[0])
            lcd.message = ("Quadrant: " + quadrant[0])
            time.sleep(2)

if __name__ == '__main__':
    camera_init(res=(1280, 720))
    
    continuous_aruco_detection()
    
    currentpos = readNumber()
    lcd.clear()
    lcd.message("Current Pos: " + currentpos)
    time.sleep(2)
    
    camera.close()
    
    

from computer_vision import detect_aruco, camera_init
from smbus2 import SMBus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

bus = SMBus(1)

address = 0x04

lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

def writeString(value): #send to arduino
    bus.write_i2c_block_data(address,0,value)
    return -1

def readString(length): #receive info from arduino
    number = bus.read_i2c_block_data(address, 0, length)
    return number

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

def send_aruco_stats():
    while True:
        marker_info = detect_aruco()
        
        if marker_info is not None:
            marker_id = marker_info['id']
            marker_angle = round(marker_info['angle'], 2)
            marker_distance = round(marker_info['distance'],3)
            
            sending = int(marker_distance) #convert distance to an integer
            send2 =  int(marker_angle)   #convert anlge value into a integer
            
            print("Marker distance: ",sending) #print to shell for debugging purposes
            print("Angle: ", send2) #print to shell for debugging purposes
            
            array = [sending,send2] #put diatnce and angle values into an array
            
            lcd.message = ("Distance: " + str(sending)) #print angle of the marker to LCD
            writeString(array) #send array to arduino
            
if __name__ == '__main__':
    camera_init()
    send_aruco_stats()
    

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
    bus.write_byte_data(address,0,value) #write byte to arduino
    return -1

def readNumber(): #receive info from arduino
    number = bus.read_byte_data(address, 0) #read byte from arduino
    return number



def send_aruco_stats():
    while True:
        marker_info = detect_aruco()
        
        if marker_info is not None:
            marker_id = marker_info['id']
            marker_angle = round(marker_info['angle'], 2)
            
            #lcd.message = ("Marker: " + str(marker_id)) #output id number to LCD
            #time.sleep(2) #wait 2 seconds
            #lcd.clear()
            
            lcd.message = ("Angle: " + str(marker_angle)) #print angle of the marker to LCD
            time.sleep(0.5) #wait 2 seconds
#        else:
#            lcd.message = "Detecting" #output id number to LCD
#            time.sleep(0.1)
#            lcd.message = ""
            
            
if __name__ == '__main__':
    camera_init()
    send_aruco_stats()
    
from computer_vision import detect_aruco
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
        marker_id = marker_info['id']
        
        if info is not None:

            writeString(marker_id) #write marker ID to arduino
            print(marker_id)
            lcd.message = ("Marker: " + str(marker_id)) #output id number to LCD
            time.sleep(2) #wait 2 seconds
            currentpos = readNumber() #read position from arduino
            lcd.clear() #clear lcd
            if currentpos == 4: #if statements to output to LCD which quadrant the wheel is in
                lcd.message = ("Curr Pos: 0") #print to LCD
                time.sleep(2) #wait 2 seconds
            elif currentpos == 3:
                lcd.message = ("Curr Pos: pi/2")
                time.sleep(2)
            elif currentpos == 2:
                lcd.message = ("Curr Pos: pi")
                time.sleep(2)       
            elif currentpos == 1:
                lcd.message = ("Curr Pos: 3*pi/2")
                time.sleep(2)
            currentpos = 0
            
            
            
if __name__ == '__main__':
    send_aruco_stats()
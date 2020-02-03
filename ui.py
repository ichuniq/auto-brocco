import serial  # Serial imported for Serial communication
import time  # Required to use delay functions
import pyautogui  # Required to to perform actions

# pyautogui.PAUSE = 2.5
pyautogui.FAILSAFE = True

ArduinoSerial = serial.Serial(
    '/dev/cu.usbmodem14201',
    9600)  #Create Serial port object called arduinoSerialData
time.sleep(2)  #wait for 2 seconds for the communication to get established

count=0
while 1:
    # read the serial data and print it as line
    incoming = str(ArduinoSerial.readline())
    print(incoming)

    if "Quit" in incoming:
        pyautogui.hotkey('command', 'w')

    if 'Switch' in incoming:
        # switch window
        pyautogui.hotkey('command', 'tab')

    if 'SwitchBack' in incoming:
        pyautogui.hotkey('command', 'shift', 'tab')

    if 'CntrlLeft' in incoming:
        pyautogui.hotkey('ctrl','left')

    if 'CntrlRight' in incoming:
        pyautogui.hotkey('ctrl','right')

    if "enter control mode" in incoming:
        if (count==0):
            count+=1;
        else :
            pyautogui.click(500, 500)
            count = 0

    if 'Volume++' in incoming:
        pyautogui.press('up')

    if 'Volume--' in incoming:
        pyautogui.press('down')

    incoming = ""

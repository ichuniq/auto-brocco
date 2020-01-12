import serial  # Serial imported for Serial communication
import time  # Required to use delay functions
import pyautogui  # Required to to perform actions

# pyautogui.PAUSE = 2.5
pyautogui.FAILSAFE = True

ArduinoSerial = serial.Serial(
    '/dev/cu.usbmodem14201',
    9600)  #Create Serial port object called arduinoSerialData
time.sleep(2)  #wait for 2 seconds for the communication to get established

while 1:
    # read the serial data and print it as line
    incoming = str(ArduinoSerial.readline())
    print(incoming)

    if 'Play/Pause' in incoming:
        pyautogui.typewrite(['space'], 0.2)

    if 'Switch' in incoming:
        # switch window
        # pyautogui.hotkey('command', 'tab')
        pyautogui.keyDown('command')
        pyautogui.press('tab')
        pyautogui.keyUp('command')

    if 'Volume++' in incoming:
        pyautogui.hotkey('up')

    if 'Volume--' in incoming:
        pyautogui.hotkey('down')

    incoming = ""

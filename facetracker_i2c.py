#!/usr/bin/env python
#Below we are importing functionality to our Code, OPEN-CV, Time, and Pimoroni Pan Tilt Hat Package of particular note.
import cv2, sys, time, os
from pantilthat import *
import smbus
import time
BetaPi = 0
GamaPi = 0
Direction = 0
CenterRange = 15


bus = smbus.SMBus(1)
time.sleep(1) #wait here to avoid 121 IO Error
address = 0x08

def writeNumber(a,b,c,d):
    bus.write_i2c_block_data(address, a, [b, c, d])
    return -1

def writeOneNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1

# Load the BCM V4l2 driver for /dev/video0. This driver has been installed from earlier terminal commands. 
#This is really just to ensure everything is as it should  be.
os.system('sudo modprobe bcm2835-v4l2')
# Set the framerate (not sure this does anything! But you can change the number after | -p | to allegedly increase or decrease the framerate).
# os.system('v4l2-ctl -p 40')
os.system('v4l2-ctl -p 40')
# Frame Size. Smaller is faster, but less accurate.
# Wide and short is better, since moving your head up and down is harder to do.
# W = 160 and H = 100 are good settings if you are using and earlier Raspberry Pi Version.
FRAME_W = 400
FRAME_H = 200

# Default Pan/Tilt for the camera in degrees. I have set it up to roughly point at my face location when it starts the code.
# Camera range is from 0 to 180. Alter the values below to determine the starting point for your pan and tilt.
cam_pan = 40
cam_tilt = 20

# Set up the Cascade Classifier for face tracking. This is using the Haar Cascade face recognition method with LBP = Local Binary Patterns. 
# Seen below is commented out the slower method to get face tracking done using only the HAAR method.
# cascPath = 'haarcascade_frontalface_default.xml' # sys.argv[1]
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

# Start and set up the video capture with our selected frame size. Make sure these values match the same width and height values that you choose at the start.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  400);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200);
time.sleep(2)

# Turn the camera to the Start position (the data that pan() and tilt() functions expect to see are any numbers between -90 to 90 degrees).
# pan(cam_pan-90)
# tilt(cam_tilt-90)
# light_mode(WS2812)

# Light control down here. If you have a LED stick wired up to the Pimoroni HAT it will light up when it has located a face.
def lights(r,g,b,w):
    for x in range(18):
        set_pixel_rgbw(x,r if x in [3,4] else 0,g if x in [3,4] else 0,b,w if x in [0,1,6,7] else 0)
    show()

# lights(0,0,0,50)

#Below we are creating an infinite loop, the system will run forever or until we manually tell it to stop (or use the "q" button on our keyboard)
while True:

    # Capture frame-by-frame
    ret, frame = cap.read()
    # This line lets you mount the camera the "right" way up, with neopixels above
    # frame = cv2.flip(frame, -1)
    
    if ret == False:
      print("Error getting image")
      continue

    # Convert to greyscale for easier+faster+accurate face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist( gray )

    # Do face detection to search for faces from these captures frames
    # cv2.CascadeClassifier.detectMultiScale(image[, scaleFactor[, minNeighbors[, flags[, minSize[, maxSize]]]]]) 
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
   
    # Slower method (this gets used only if the slower HAAR method was uncommented above. 
    '''faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=4,
        minSize=(20, 20),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE | cv2.cv.CV_HAAR_FIND_BIGGEST_OBJECT | cv2.cv.CV_HAAR_DO_ROUGH_SEARCH
    )'''
    
    # lights(50 if len(faces) == 0 else 0, 50 if len(faces) > 0 else 0,0,50)

    #Below draws the rectangle onto the screen then determines how to move the camera module so that the face can always be in the centre of screen. 

    for (x, y, w, h) in faces:
        # Draw a green rectangle around the face (There is a lot of control to be had here, for example If you want a bigger border change 4 to 8)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)

        # Track face with the square around it
        
        # Get the centre of the face
        x = x + (w/2)
        y = y + (h/2)
        # Center the coordinates in the middle, left positive, right negative
        z = float(x - (FRAME_W/2))
        t = float(y - (FRAME_H/2))
        
        # Converts pixels to degrees, the usb camera has a 120 degrees angle
        # proprtionally devided to the 400 pixels of the image width
        # since we can just send 0-255 via i2c, we will Center the angle
        # around 100
              
        p = int (0.1 * z * 120/400+100)
        
        if -CenterRange < z < CenterRange and -CenterRange < t < CenterRange:
            Direction = 0
        if -CenterRange < z < CenterRange and CenterRange <= t :
            Direction = 1
        if CenterRange <= z and CenterRange <= t :
            Direction = 2
        if CenterRange <= z and -CenterRange < t < CenterRange :
            Direction = 3       
        if CenterRange <= z and t <= -CenterRange :
            Direction = 4            
        if -CenterRange < z < CenterRange and t <= -CenterRange :
            Direction = 5
        if z <= -CenterRange and t <= -CenterRange :
            Direction = 6
        if z <= -CenterRange and -CenterRange < t < CenterRange :
            Direction = 7
        if z <= -CenterRange and CenterRange <= t :
            Direction = 8
            
            
            
        if -20 >= z:
             BetaPi = int (0)
        if -20 < z < 20:
             BetaPi = int (1)
        if  20 <= z:
             BetaPi = int (2)
             
        if -20 >= t:
             GamaPi = int (0)
        if -20 < t < 20:
             GamaPi = int (1)
        if  20 <= t:
             GamaPi = int (2)             
        # p = int (0.1 * z * 120/400)
        # print(x,y,z,p)
        # print(BetaPi, GamaPi)
        print(Direction)
        
        writeOneNumber(Direction)
        # writeOneNumber(p)
        # writeNumber(100,p,100,10)
        time.sleep(0.1)                    #delay one second


        # Correct relative to centre of image
        # turn_x  = float(x - (FRAME_W/2))
        # turn_y  = float(y - (FRAME_H/2))

        # Convert to percentage offset
        # turn_x  /= float(FRAME_W/2)
        # turn_y  /= float(FRAME_H/2)

        # Scale offset to degrees (that 2.5 value below acts like the Proportional factor in PID)
        # turn_x   *= 2.5 # VFOV
        # turn_y   *= 2.5 # HFOV
        # cam_pan  += -turn_x
        # cam_tilt += turn_y

        # print(cam_pan-90, cam_tilt-90)

        # Clamp Pan/Tilt to 0 to 180 degrees
        # cam_pan = max(0,min(180,cam_pan))
        # cam_tilt = max(0,min(180,cam_tilt))

        # Update the servos
        # pan(int(cam_pan-90))
        # tilt(int(cam_tilt-90))

        break
    
    #Orientate the frame so you can see it.
    # frame = cv2.resize(frame, (540,300))
    frame = cv2.resize(frame, (400,200))
    frame = cv2.flip(frame, 1)
   
    # Display the video captured, with rectangles overlayed
    # onto the Pi desktop 
    cv2.imshow('Video', frame)

    #If you type q at any point this will end the loop and thus end the code.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture information and stop everything
video_capture.release()
cv2.destroyAllWindows()

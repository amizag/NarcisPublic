#!/usr/bin/env python
#Below we are importing functionality to our Code, OPEN-CV, Time, and Pimoroni Pan Tilt Hat Package of particular note.
import cv2, sys, time, os
# from pantilthat import *
import smbus
import time
from adafruit_servokit import ServoKit
from varspeed import Vspeed
import numpy as np

# no need for this now, using the adafruit library to talk to the hat now
# bus = smbus.SMBus(1)
# time.sleep(1) #wait here to avoid 121 IO Error
# address = 0x08

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

BetaPi = 0
GamaPi = 0
Direction = 0
CenterRange = 15
AngleIncrement = 1

MIN0 = 20
MAX0 = 80
MIN1 = 14
MAX1 = 80
MIN2 = 12
MAX2 = 80
MaxArmHeight = 142

AbsPositionBeta = 0;
AbsPositionGama = 0;
ActualPositionBeta = 0;
ActualPositionGama = 0;

NoFaceCount = 0

# Readjusts the pulsewidth for 0 and 180 for each motor
kit.servo[0].set_pulse_width_range(500, 1900)
kit.servo[1].set_pulse_width_range(500, 1900)
kit.servo[2].set_pulse_width_range(500, 1900)

# Move the three servos to their innitial position
kit.servo[0].angle = MIN0+40
kit.servo[1].angle = MIN1+40
kit.servo[2].angle = MIN2+40

# Give it time to move
time.sleep(3)

# init_position = initial start position
# result = float, int
vs0 = Vspeed(init_position=MIN0, result="int")
vs1 = Vspeed(init_position=MIN1, result="int")
vs2 = Vspeed(init_position=MIN2, result="int")

# make the output of the function be within the bounds set
vs0.set_bounds(lower_bound=MIN0, upper_bound=MAX0)
vs1.set_bounds(lower_bound=MIN1, upper_bound=MAX1)
vs2.set_bounds(lower_bound=MIN2, upper_bound=MAX2)


def Move_mirror(mirrorNum, Height, Beta, Gama, Speed):
    # pass
    R = 58; # servo arm length
    Hcst = 27.0; # The height constante. 27mm here
    Speed = 10

    InitAngle0 = 9.0;  # the angle that the servo arm makes with the horizon when the servo is at 0deg
    InitAngle1 = 15.0;  
    InitAngle2 = 17.0;
    
    L = Height
    
    W = 156.0; # The horizontal distance between the two L and R joint 
    X = 136.0; # The vertical distance between the upper and lower joints  

    Beta = np.clip(Beta, -20, 20) 
    Gama = np.clip(Gama, -20, 20)
     
    BetaRad = Beta * 3.14 / 180.0;  
    LdifLR = W * np.sin(BetaRad) / 2.0; # the difference between left and right arms
  
    GamaRad = Gama * 3.14 / 180.0;  
    LdifUD = X * np.sin(GamaRad) / 2.0; # the difference between Up and dowm arms
  
    LOne = L - LdifUD; 
    LTwo = L - LdifLR + LdifUD;
    LThree = L + LdifLR + LdifUD;

    MotorAngle0 = 57.324 * np.arcsin((LOne-Hcst)/(2*R)) - InitAngle0;
    MotorAngle1 = 57.324 * np.arcsin((LTwo-Hcst)/(2*R)) - InitAngle1;
    MotorAngle2 = 57.324 * np.arcsin((LThree-Hcst)/(2*R)) - InitAngle2;
    
    running0 = True
    running1 = True
    running2 = True
    while running1: # and running1 and running2:
        # move(new_position,time_secs of move,steps in move,easing function
        # move from the current position
        position0, running0, changed0 = vs0.move(new_position=MotorAngle0, time_secs=0.2, steps=200, easing="LinearInOut")
        if changed0:  # only act if the output changed
            kit.servo[0].angle = position0

        position1, running1, changed1 = vs1.move(new_position=MotorAngle1, time_secs=0.2, steps=200, easing="LinearInOut")
        if changed1:  # only act if the output changed

            kit.servo[1].angle = position1
            
        position2, running2, changed2 = vs2.move(new_position=MotorAngle2, time_secs=0.2, steps=200, easing="LinearInOut")
        if changed2:  # only act if the output changed
            kit.servo[2].angle = position2
        
    # print(f'End of function call')
    
def Move_mirror_high(mirrorNum, Height, Beta, Gama, Duration):
    # pass
    R = 58; # servo arm length
    Hcst = 27.0; # The height constante. 27mm here
    Speed = 10

    InitAngle0 = 9.0;  # the angle that the servo arm makes with the horizon when the servo is at 0deg
    InitAngle1 = 15.0;  
    InitAngle2 = 17.0;
    
    L = 100
    
    W = 156.0; # The horizontal distance between the two L and R joint 
    X = 136.0; # The vertical distance between the upper and lower joints  

    Beta = np.clip(Beta, -20, 20) 
    Gama = np.clip(Gama, -20, 20)
     
    BetaRad = Beta * 3.14 / 180.0;  
    LdifLR = W * np.sin(BetaRad) / 2.0; # the difference between left and right arms
  
    GamaRad = Gama * 3.14 / 180.0;  
    LdifUD = X * np.sin(GamaRad) / 2.0; # the difference between Up and dowm arms
  
    LOne = L - LdifUD; 
    LTwo = L - LdifLR + LdifUD;
    LThree = L + LdifLR + LdifUD;
    
    highest = max(LOne,LTwo,LThree)
    Dif = Height - highest
    
    LOne = LOne + Dif
    LTwo = LTwo + Dif
    LThree = LThree + Dif
    
    MotorAngle0 = 57.324 * np.arcsin((LOne-Hcst)/(2*R)) - InitAngle0;
    MotorAngle1 = 57.324 * np.arcsin((LTwo-Hcst)/(2*R)) - InitAngle1;
    MotorAngle2 = 57.324 * np.arcsin((LThree-Hcst)/(2*R)) - InitAngle2;
    
    running0 = True
    running1 = True
    running2 = True
    while running1: # and running1 and running2:
        # move(new_position,time_secs of move,steps in move,easing function
        # move from the current position
        position0, running0, changed0 = vs0.move(new_position=MotorAngle0, time_secs=Duration, steps=(Duration*200), easing="LinearInOut")
        if changed0:  # only act if the output changed
            kit.servo[0].angle = position0

        position1, running1, changed1 = vs1.move(new_position=MotorAngle1, time_secs=Duration, steps=(Duration*200), easing="LinearInOut")
        if changed1:  # only act if the output changed

            kit.servo[1].angle = position1
            
        position2, running2, changed2 = vs2.move(new_position=MotorAngle2, time_secs=Duration, steps=(Duration*200), easing="LinearInOut")
        if changed2:  # only act if the output changed
            kit.servo[2].angle = position2
        
    # print(f'End of function call')
    
def Direct_Move_mirror_high(mirrorNum, Height, Beta, Gama):
    # pass
    R = 58; # servo arm length
    Hcst = 27.0; # The height constante. 27mm here
    Speed = 10

    InitAngle0 = 9.0;  # the angle that the servo arm makes with the horizon when the servo is at 0deg
    InitAngle1 = 15.0;  
    InitAngle2 = 17.0;
    
    L = 100
    
    W = 156.0; # The horizontal distance between the two L and R joint 
    X = 136.0; # The vertical distance between the upper and lower joints  

    Beta = np.clip(Beta, -20, 20) 
    Gama = np.clip(Gama, -20, 20)
     
    BetaRad = Beta * 3.14 / 180.0;  
    LdifLR = W * np.sin(BetaRad) / 2.0; # the difference between left and right arms
  
    GamaRad = Gama * 3.14 / 180.0;  
    LdifUD = X * np.sin(GamaRad) / 2.0; # the difference between Up and dowm arms
  
    LOne = L - LdifUD; 
    LTwo = L - LdifLR + LdifUD;
    LThree = L + LdifLR + LdifUD;
    
    highest = max(LOne,LTwo,LThree)
    Dif = Height - highest
    
    LOne = LOne + Dif
    LTwo = LTwo + Dif
    LThree = LThree + Dif
    
    MotorAngle0 = 57.324 * np.arcsin((LOne-Hcst)/(2*R)) - InitAngle0;
    MotorAngle1 = 57.324 * np.arcsin((LTwo-Hcst)/(2*R)) - InitAngle1;
    MotorAngle2 = 57.324 * np.arcsin((LThree-Hcst)/(2*R)) - InitAngle2;
    
    kit.servo[0].angle = MotorAngle0
    kit.servo[1].angle = MotorAngle1
    kit.servo[2].angle = MotorAngle2

        
    # print(f'End of function call')
    
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
    # print(faces.size)
    if len(faces) == 0:
        NoFaceCount = NoFaceCount + 1
        print("No Face", NoFaceCount)
        if NoFaceCount > 80:
            Move_mirror_high(0, MaxArmHeight,   0,  0, 0.5)
    else:
        NoFaceCount = 0
        print("Face Detected")
    # Slower method (this gets used only if the slower HAAR method was uncommented above. 
    '''faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=4,
        minSize=(20, 20),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE | cv2.cv.CV_HAAR_FIND_BIGGEST_OBJECT | cv2.cv.CV_HAAR_DO_ROUGH_SEARCH
    )'''

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
        ErrorDegBeta = int (1 * z * 120/400)
        ErrorDegGama = int (1 * t * 120/200)
        
        if -CenterRange < z < CenterRange and -CenterRange < t < CenterRange:
            Direction = 0
            deltaBeta = 0
            deltaGama = 0  
        if -CenterRange < z < CenterRange and CenterRange <= t :
            Direction = 1
            deltaBeta = 0              
            deltaGama = AngleIncrement
        if CenterRange <= z and CenterRange <= t :
            Direction = 2
            deltaBeta = AngleIncrement
            deltaGama = AngleIncrement
        if CenterRange <= z and -CenterRange < t < CenterRange :
            Direction = 3
            deltaBeta = AngleIncrement
            deltaGama = 0 
        if CenterRange <= z and t <= -CenterRange :
            Direction = 4
            deltaBeta = AngleIncrement
            deltaGama = -AngleIncrement
        if -CenterRange < z < CenterRange and t <= -CenterRange :
            Direction = 5
            deltaBeta = 0
            deltaGama = -AngleIncrement
        if z <= -CenterRange and t <= -CenterRange :
            Direction = 6
            deltaBeta = -AngleIncrement
            deltaGama = -AngleIncrement
        if z <= -CenterRange and -CenterRange < t < CenterRange :
            Direction = 7
            deltaBeta = -AngleIncrement
            deltaGama = 0
        if z <= -CenterRange and CenterRange <= t :
            Direction = 8
            deltaBeta = -AngleIncrement
            deltaGama = AngleIncrement
            
        # the function moving the motors work in absolute positions, converting relative to absolue here
        AbsPositionBeta = ActualPositionBeta - deltaBeta; 
        AbsPositionGama = ActualPositionGama - deltaGama;
        
        # Call the function that will move the motors
        Move_mirror_high(0, MaxArmHeight,   - AbsPositionBeta,  -AbsPositionGama, 0.01)
        
        # setting the new Actual position
        ActualPositionBeta = AbsPositionBeta;
        ActualPositionGama = AbsPositionGama;
        
        time.sleep(0.05)                    #delay one second

        break
    
    #Orientate the frame so you can see it.
    
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

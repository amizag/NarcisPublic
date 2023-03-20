import time
from adafruit_servokit import ServoKit
from varspeed import Vspeed
import numpy as np

MIN0 = 20
MAX0 = 80
MIN1 = 14
MAX1 = 80
MIN2 = 12
MAX2 = 80

MaxArmHeight = 142

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

# Readjusts the pulsewidth for 0 and 180 for each motor

kit.servo[0].set_pulse_width_range(500, 1900)
kit.servo[1].set_pulse_width_range(500, 1900)
kit.servo[2].set_pulse_width_range(500, 1900)

# Move the three servos to their innitial position
kit.servo[0].angle = MIN0
kit.servo[1].angle = MIN1
kit.servo[2].angle = MIN2

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

# def clamp(n, minn, maxn):
#     return max(min(maxn, n), minn)

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

Move_mirror_high(0, MaxArmHeight,   0,  0, 1)
Move_mirror_high(0, MaxArmHeight,   0, 20, 1)
Move_mirror_high(0, MaxArmHeight,  20, 20, 1)
Move_mirror_high(0, MaxArmHeight,  20,  0, 1)
Move_mirror_high(0, MaxArmHeight,  20,-20, 1)
Move_mirror_high(0, MaxArmHeight,   0,-20, 1)
Move_mirror_high(0, MaxArmHeight, -20,-20, 1)
Move_mirror_high(0, MaxArmHeight, -20,  0, 1)
Move_mirror_high(0, MaxArmHeight,   0,  0, 1)

# Move_mirror(0, 110, 0, 0, 20)
# time.sleep(1)
# Move_mirror(0, 110,  10, 0, 20)
# time.sleep(1)
# Move_mirror(0, 110,  -10, 0, 20)
# time.sleep(1)
# Move_mirror(0, 130,  0, 0, 20)
# time.sleep(1)
# Move_mirror(0, 140,  0, 0, 20)
# time.sleep(1)
# Move_mirror(0, 110,  0, 0, 20)
# Move_mirror(0, 110,  0, 10, 20)
# Move_mirror(0, 110,  10, 10, 20)
# Move_mirror(0, 110,  10, 0, 20)
# Move_mirror(0, 110,  10, -10, 20)
# Move_mirror(0, 110,  0, -10, 20)
# Move_mirror(0, 110,  -10, -10, 20)
# Move_mirror(0, 110,  -10, 0, 20)
# Move_mirror(0, 110,  0, 0, 20)
print(f'End of SCRIPT')    


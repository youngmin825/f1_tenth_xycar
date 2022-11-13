import math

speed = 0
angle = 3

if(3 >= 2):
    speed = 3
elif(3 == 3):


if(angle >= -10 / 180 * math.pi or angle <= 10 / 180 * math.pi ):
    speed = 1.5
else if(((angle >= -20 / 180 * math.pi) and (angle <= -10 / 180 * math.pi)) or ((angle <= 20 / 180 * math.pi) and (angle >= 10 / 180 * math.pi))):
    speed = 1.0
else:
    speed = 0.5
#!/usr/bin/env python
import math as m

# system params
width =  640.0 #320.0
height = 480.0 #240.0
hfov = 1.047

focal_length = (width/2) * m.tan(hfov/2)
print("focal length of our gazebo camera is: {}".format(focal_length))

K= [[554.3827128226441, 0.0, 320.5], [0.0, 554.3827128226441, 240.5], [0.0, 0.0, 1.0]]
print("Camera calibration matrix is: ")
for i in range(3):
    for j in range(3):
        print(K[i][j]),
    print(' ')

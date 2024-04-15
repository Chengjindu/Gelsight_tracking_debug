from lib import find_marker
import numpy as np
import cv2
import time
import marker_dectection
import sys
import os
import setting

calibrate = False

if len(sys.argv) > 1:
    if sys.argv[1] == 'calibrate':
        calibrate = True

gelsight_version = 'Bnz'
# gelsight_version = 'HSR'

# cap = cv2.VideoCapture("data/GelSight_Twist_Test.mov")
cap = cv2.VideoCapture("data/GelSight_Shear_Test.mov")
# cap = cv2.VideoCapture(1)


# Resize scale for faster image processing
setting.init()
RESCALE = setting.RESCALE

# Create Mathing Class (this process needs to be automated)
m = find_marker.Matching(
    N_=setting.N_, 
    M_=setting.M_, 
    fps_=setting.fps_, 
    x0_=setting.x0_, 
    y0_=setting.y0_, 
    dx_=setting.dx_, 
    dy_=setting.dy_)
"""
N_, M_: the row and column of the marker array
x0_, y0_: the coordinate of upper-left marker
dx_, dy_: the horizontal and vertical interval between adjacent markers
"""

# save video
fourcc = cv2.VideoWriter_fourcc(*'XVID')

if gelsight_version == 'HSR':
    out = cv2.VideoWriter('output.mp4',fourcc, 30.0, (215,215))
else:
    out = cv2.VideoWriter('output.mp4',fourcc, 30.0, (1280//RESCALE,720//RESCALE))

while(True):

    # capture frame-by-frame
    ret, frame = cap.read()
    if not(ret):
        break

    frame_raw = frame.copy()
    # cv2.imshow('raw',frame_raw)
    # cv2.waitKey(0)

    # resize (or unwarp)
    if gelsight_version == 'HSR':
        frame = marker_dectection.init_HSR(frame)
        cv2.imshow('HSR', frame) 
        cv2.waitKey(0)
    else:
        frame = marker_dectection.init(frame)
        # cv2.imshow('HSR', frame) 
        # cv2.waitKey(0)
    # frame = marker_dectection.init_HSR(frame)

    # find marker masks
    mask = marker_dectection.find_marker(frame)
    # cv2.imshow('mask', mask) 
    # cv2.waitKey(0)

    # find marker centers and store as an array
    mc = marker_dectection.marker_center(mask, frame)
    # cv2.imshow('mask', mask) 
    # cv2.waitKey(0)

    if calibrate == False:
        tm = time.time()
        # matching init
        m.init(mc)

        # matching
        m.run()
        print(time.time() - tm)

        # matching result
        """
        output: (Ox, Oy, Cx, Cy, Occupied) = flow
            Ox, Oy: N*M matrix, the x and y coordinate of each marker at frame 0
            Cx, Cy: N*M matrix, the x and y coordinate of each marker at current frame
            Occupied: N*M matrix, the index of the marker at each position, -1 means inferred. 
                e.g. Occupied[i][j] = k, meaning the marker mc[k] lies in row i, column j.
        """
        flow = m.get_flow()

        # draw flow
        marker_dectection.draw_flow(frame, flow)

    mask_img = mask.astype(frame[0].dtype)
    mask_img = cv2.merge((mask_img, mask_img, mask_img))

    # cv2.imshow('raw',frame_raw)
    cv2.imshow('frame',frame)

    if calibrate:
        # Display the mask 
        cv2.imshow('mask',mask_img)

    out.write(frame)

    print(frame.shape)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
out.release()
cv2.destroyAllWindows()
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
fig = plt.figure()

## If you want to generate multiple markers
nx = 1
ny = 1
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,1, 700) ### dictionary, id, sidePixels
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

plt.savefig("_data/markers.jpg")
plt.show()
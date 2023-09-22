# -*- coding: utf-8 -*-
"""
Created on Sun Jan 15 19:10:39 2023

@author: 93982
"""

from __future__ import print_function
print('start')

import numpy as np
import laspy
import copy
import open3d 
import matplotlib.pyplot as plt

print('end')

print("start")
#cloud = pcl.load(file)
print('end')


input="./training/labelledLocalCRS/DEBY_LOD2_4959462/ransac_3.pcd"
    
pcd = open3d.io.read_point_cloud(input)
pcd = open3d.geometry.PointCloud(pcd)

# remove black points
color = np.asarray(pcd.colors) #n*3
print(color)
print(len(color))

black=[]
for i in range(len(color)):
    if color[i].any()==0:
        black.append(i)

nonblack = pcd.select_by_index(black, invert=True)

open3d.visualization.draw_geometries([nonblack])
open3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/nonblack.pcd", nonblack)






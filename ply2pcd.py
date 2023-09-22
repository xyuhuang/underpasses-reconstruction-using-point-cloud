# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 19:18:11 2023

@author: 93982
"""

import open3d as o3d

file='./training/labelledLocalCRS/DEBY_LOD2_4959462/right_t.ply'
#file='./training/labelledLocalCRS/DEBY_LOD2_4959462/cropped_462_1.ply'
print("start")

pcd = o3d.io.read_point_cloud(file)
o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/right_t.pcd", pcd)
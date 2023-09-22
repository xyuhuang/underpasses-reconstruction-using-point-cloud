# -*- coding: utf-8 -*-
"""
Created on Mon Jan  9 18:20:13 2023

@author: 93982
"""
from __future__ import print_function

import sys
#'C:\\Users\\93982\\Downloads\\python-pcl快速配置系统\\example'
paths=[ 'D:\\Anaconda\\envs\\py36\\python36.zip', 'D:\\Anaconda\\envs\\py36\\DLLs', 'D:\\Anaconda\\envs\\py36\\lib', 'D:\\Anaconda\\envs\\py36', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\python_pcl-0.3.0rc1-py3.6-win-amd64.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\nose-1.3.7-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\mock-5.0.0-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\filelock-3.9.0-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\win32', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\win32\\lib', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\Pythonwin']

for p in paths:
    if not p in sys.path:
        sys.path.append(p)


import numpy as np
#import pcl
#import pcl.pcl_visualization
import open3d as o3d


file='./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis_rough.pcd'
#file='./training/labelledLocalCRS/DEBY_LOD2_4959462/nonblack.pcd'
#file='./training/labelledLocalCRS/DEBY_LOD2_4959462/cropped_462_1.pcd'

def readO3D():
    pcd = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries([pcd])
    #o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.ply", pcd)

readO3D()


# def readCloud():

#     cloud = pcl.load('./training/labelledLocalCRS/DEBY_LOD2_4959462/cluster_1.pcd')
#     # Centred the data
#     centred = cloud - np.mean(cloud, 0)
#     # print(centred)
#     ptcloud_centred = pcl.PointCloud()
#     ptcloud_centred.from_array(centred)
#     visual = pcl.pcl_visualization.CloudViewing()

#     # PointXYZ
#     visual.ShowMonochromeCloud(ptcloud_centred, b'cloud')
#     v = True
#     while v:
#         v = not(visual.WasStopped())


#readCloud()
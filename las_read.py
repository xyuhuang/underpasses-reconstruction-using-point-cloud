# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 12:04:22 2023

@author: 93982
"""
# -*- coding: utf-8 -*-
# Point cloud library
from __future__ import print_function

####add path to spyder
import sys
#'C:\\Users\\93982\\Downloads\\python-pcl快速配置系统\\example'
paths=[ 'D:\\Anaconda\\envs\\py36\\python36.zip', 'D:\\Anaconda\\envs\\py36\\DLLs', 'D:\\Anaconda\\envs\\py36\\lib', 'D:\\Anaconda\\envs\\py36', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\python_pcl-0.3.0rc1-py3.6-win-amd64.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\nose-1.3.7-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\mock-5.0.0-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\filelock-3.9.0-py3.6.egg', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\win32', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\win32\\lib', 'D:\\Anaconda\\envs\\py36\\lib\\site-packages\\Pythonwin']

for p in paths:
    if not p in sys.path:
        sys.path.append(p)
####

#import pcl
#import pcl.pcl_visualization
import numpy as np
import laspy
from laspy.file import File 
#from copy import deepcopy
#from pclpy import pcl
from pclpy import pcl
import pclpy

import pcl
#from pcl import pcl_visualization
#import pcl.pcl_visualization
import matplotlib.pyplot as plt

import open3d as o3d

#convert las file to pcd file
def las2pcd():
    input_file="./training/labelledGlobalCRS/DEBY_LOD2_4959462.las"
    point_cloud=laspy.read(input_file)
    
    points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()
    colors = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()
    #inFile = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z,point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()
    # cloud = pcl.PointCloud_PointXYZRGB().from_array(np.array(inFile, dtype=np.float32))
    cloud = pcl.PointCloud.PointXYZRGBA().from_array(np.array(points, dtype=np.float32),colors)
    # address="D:/TUM File/semester3/photogrammetry project/data/training/labelledGlobalCRS/DEBY_LOD2_4959462_pcl.pcd"
    # pcl.io.savePCDFileASCII(address,cloud)
    
    # f = laspy.read(input_file)
    # inFile = np.vstack((f.x, f.y, f.z, f.intensity)).transpose()
    # cloud = pcl.PointCloud__PointXYZRGB()
    # cloud.from_array(np.array(inFile, dtype=np.float32))
    #f.close()
    address="./training/labelledGlobalCRS/DEBY_LOD2_4959462_pcl.pcd"
    pcl.io.savePCDFileASCII(address,cloud)
    #pcl.save(cloud, address)
    #return cloud

    

#visualize pcd file
def readCloud(): 
    file='./training/labelledGlobalCRS/DEBY_LOD2_4959462_pcl.pcd'
    #file='D:/TUM File/semester3/photogrammetry project/data/training/labelledGlobalCRS/DEBY_LOD2_4959462_pcl.pcd'
    ######## cloud = pcl.load(file)
    cloud = o3d.io.read_point_cloud(file)
    # Centred the data
    cloud = np.asarray(cloud.points)
    centred = cloud - np.mean(cloud, 0)
    # print(centred)
    ptcloud_centred = pcl.PointCloud()
    ptcloud_centred.from_array(centred)
    visual = pcl.pcl_visualization.CloudViewing()

    # PointXYZ
    visual.ShowMonochromeCloud(ptcloud_centred, b'cloud')
    v = True
    while v:
        v = not(visual.WasStopped())
        
#las2pcd()       
readCloud()












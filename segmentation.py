# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 18:53:33 2023

@author: 93982
"""
from __future__ import print_function
print('start')

import pcl
#import pcl.pcl_visualization
import numpy as np
import laspy
import copy
import open3d
import matplotlib.pyplot as plt

print('end')


#file='./training/labelledLocalCRS/DEBY_LOD2_4959462/cropped_462.pcd'
file='./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_462_2.pcd'

print("start")
#cloud = pcl.load(file)
print('end')


#RANSAC
def open3d_segment():
    # read cloud
    input="./training/labelledLocalCRS/DEBY_LOD2_4959462/cluster_voxel_1.pcd"
    
    pcd = open3d.io.read_point_cloud(input)
    pcd = open3d.geometry.PointCloud(pcd)
    #pcd.paint_uniform_color(color=[0.5, 0.5, 0.5])
    
    ####1
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.4, 
                                             ransac_n=10, 
                                             num_iterations=1000)
    [A, B, C, D] = plane_model
    print(f"Plane equation: {A:.2f}x + {B:.2f}y + {C:.2f}z + {D:.2f} = 0")
    
    colors = np.array(pcd.colors)
    colors[inliers] = [0.9, 0.9, 0.98]  # set point in the plane purple
    ####
    
    ####2
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    plane_model2, inliers2 = outlier_cloud.segment_plane(distance_threshold=0.8, 
                                                         ransac_n=10, 
                                                         num_iterations=1000)
    [A2, B2, C2, D2] = plane_model
    print(f"Plane equation: {A2:.2f}x + {B2:.2f}y + {C2:.2f}z + {D2:.2f} = 0")
    colors[inliers2] = [1, 0.75, 0.79]  #pink
    ####
    
    ####3
    # outlier_cloud2 = outlier_cloud.select_by_index(inliers2, invert=True)
    # plane_model3, inliers3 = outlier_cloud2.segment_plane(distance_threshold=0.4, 
    #                                                      ransac_n=10, 
    #                                                      num_iterations=1000)
    # [A3, B3, C3, D3] = plane_model
    # print(f"Plane equation: {A3:.2f}x + {B3:.2f}y + {C3:.2f}z + {D3:.2f} = 0")
    # colors[inliers3] = [0.9, 0.9, 0.98]  #purple
    ####
    
    
    pcd.colors = open3d.utility.Vector3dVector(colors)
    # visulization
    open3d.visualization.draw_geometries([pcd],
                                         window_name="segment",
                                         width=800,
                                         height=600)
    open3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/ransac_3.pcd", pcd)
    

open3d_segment()


    


# DBSCAN Clustering
def open3d_cluster():
    # 读取点云文件
    #pcd_path = r"E:\Study\Machine Learning\Dataset3d\points_pcd\cat.pcd"
    pcd = open3d.io.read_point_cloud(file)
    print(pcd)
    pcd = open3d.geometry.PointCloud(pcd)
    # 聚类距离设置为4，组成一类至少需要20个点
    #0.13 600
    labels = pcd.cluster_dbscan(eps=0.15, min_points=200, print_progress=True)
    max_label = max(labels)
    print(max_label)
    # 随机构建n+1种颜色，这里需要归一化
    colors = np.random.randint(1, 255, size=(max_label + 1, 3)) / 255.
    colors = colors[labels]             # 每个点云根据label确定颜色
    colors[np.array(labels) < 0] = 0    # 噪点配置为黑色
    pcd.colors = open3d.utility.Vector3dVector(colors)  # 格式转换(由于pcd.colors需要设置为Vector3dVector格式)
    # 可视化点云列表
    open3d.visualization.draw_geometries([pcd],
                                         window_name="cluster",
                                         width=800,
                                         height=600)
    open3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/cluster_voxel_2.pcd", pcd)

    
#open3d_cluster()    















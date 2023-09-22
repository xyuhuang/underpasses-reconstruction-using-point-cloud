# -*- coding: utf-8 -*-
"""
Created on Sat Jan 14 16:51:50 2023

@author: 93982
"""

from __future__ import print_function

import numpy as np
import laspy
import copy
import open3d as o3d
import matplotlib.pyplot as plt


file="./training/labelledLocalCRS/DEBY_LOD2_4959462/cropped_462_1.pcd"
#file="./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_462_0.pcd"

def voxelize():
    # print('input')
    # N = 2000
    # pcd = o3dtut.get_armadillo_mesh().sample_points_poisson_disk(N)
    # # fit to unit cube
    # pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
    #           center=pcd.get_center())
    # pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    # o3d.visualization.draw_geometries([pcd])
    
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    #pcd = open3d.geometry.PointCloud(pcd)
    print('voxelization')
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                                voxel_size=0.5)
    o3d.visualization.draw_geometries([voxel_grid])
    o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_462_1.pcd", pcd)
    print(pcd)
    

#voxelize()


def voxelDownSampling():
    print("Downsample the point cloud with a voxel of 0.03")
    pcd=o3d.io.read_point_cloud(file)
    print(pcd)
    downpcd = pcd.voxel_down_sample(voxel_size=0.03)
    o3d.visualization.draw_geometries([downpcd])
    #write
    o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_462_2.pcd", downpcd)
    print(downpcd)

voxelDownSampling()







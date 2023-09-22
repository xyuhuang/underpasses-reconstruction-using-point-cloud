# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 12:20:33 2023

@author: 93982
"""

#pcd_combined = o3d.geometry.PointCloud()
#pcds = load_point_clouds([pcd_EulerAngle,pcd_EulerAngle2])
import open3d as o3d
import numpy as np

input_ground="./training/labelledLocalCRS/DEBY_LOD2_4959462/ground_regis.pcd"
input_roof="./training/labelledLocalCRS/DEBY_LOD2_4959462/roof_regis.pcd"
input_wall="./training/labelledLocalCRS/DEBY_LOD2_4959462/wall_regis.pcd"

# ---------------将两个点云进行拼接------------------
pcd_ground = o3d.io.read_point_cloud(input_ground)
pcd_roof = o3d.io.read_point_cloud(input_roof)
pcd_wall = o3d.io.read_point_cloud(input_wall)
pcd_connect = o3d.geometry.PointCloud()
pcd_connect.points = o3d.utility.Vector3dVector(np.concatenate((np.asarray(pcd_ground.points),
                                                                np.asarray(pcd_roof.points),
                                                                np.asarray(pcd_wall.points))))  # 拼接点云


o3d.visualization.draw_geometries([pcd_connect])
o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.pcd",pcd_connect)
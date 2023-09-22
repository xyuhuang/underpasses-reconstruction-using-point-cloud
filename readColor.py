# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 12:03:43 2023

@author: 93982
"""
import open3d 
import numpy as np

# read cloud color
input="./training/labelledLocalCRS/DEBY_LOD2_4959462/nonblack.pcd"
    
pcd = open3d.io.read_point_cloud(input)
pcd = open3d.geometry.PointCloud(pcd)
#np.set_printoptions(precision=2)
colors = np.array(pcd.colors)
colors=np.around(colors, decimals=2)

print(type(colors))
print(colors[0])
print(type(colors[0]))

uniques = np.unique(colors,axis=0)
print(uniques)

m=np.shape(colors)
print(m)
print(type(m))
n=m[0]  #number of point cloud
print(n)
groundcolor=np.array([0.9, 0.9, 0.98])
groundi=[]


for i in range(n):
    if (colors[i] == [0.9, 0.9, 0.98]).all():
        groundi.append(i)
        
print(groundi)       
ground=pcd.select_by_index(groundi)        
open3d.visualization.draw_geometries([ground])
open3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/ground_source.pcd", ground)

# pcd9 = o3d.io.read_point_cloud('fragment_018.pcd')
# vis = o3d.visualization.VisualizerWithEditing()
# vis.create_window()
# vis.add_geometry(pcd9)
# # 激活窗口。此函数将阻止当前线程，直到窗口关闭。
# vis.run()  # 等待用户拾取点   shift +鼠标左键
# vis.destroy_window()










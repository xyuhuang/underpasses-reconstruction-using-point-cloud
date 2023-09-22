# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 19:04:15 2023

@author: 93982
"""

 
import numpy as np
import copy
import open3d as o3d
 
#file='./training/labelledGlobalCRS/DEBY_LOD2_4959462_pcl.pcd'
file='./training/labelledLocalCRS/DEBY_LOD2_4959462/cropped_462.pcd'
print("start")
 
def demo_crop_geometry():
    print("手动裁剪点云示例")
 
    print("按键 K 锁住点云，并进入裁剪模式")
    print("用鼠标左键拉一个矩形框选取点云，或者用 《ctrl+左键单击》 连线形成一个多边形区域")
    print("按键 C 结束裁剪并保存点云")
    print("按键 F 解除锁定，恢复自由查看点云模式")
    print("17873123809")
    pcd = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries_with_editing([pcd])
 
 
 
demo_crop_geometry()
 
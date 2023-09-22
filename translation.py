# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 12:06:56 2023

@author: yangx
"""

import copy  # 点云深拷贝
import open3d as o3d
import numpy as np

# -------------------------- 加载点云 ------------------------
print("->正在加载点云... ")
pcd = o3d.io.read_point_cloud("nonblack_2.pcd")
pcd_target = o3d.io.read_point_cloud("ground.pcd")
print(pcd)
print(f'pcd质心：{pcd.get_center()}')
# ===========================================================

# -------------------------- 点云平移 ------------------------
print("\n->沿X轴平移0.2m")
pcd_tx = copy.deepcopy(pcd_target).translate((0.2, 0.2, 0.2))
pcd_tx.paint_uniform_color([1, 0, 0])
print(pcd_tx)
print(f'pcd_tx质心：{pcd_tx.get_center()}')

print("\n->将点云质心平移到指定位置")
pcd_new = copy.deepcopy(pcd_tx).translate((-21.7, -8.4, 24.1),relative=True)
#pcd_new = copy.deepcopy(pcd_tx).translate((0.2, 0.2, 0.2),False)  # relative 可以省略
pcd_new.paint_uniform_color([0, 1, 0])
print(pcd_new)
print(f'pcd_new：{pcd_new.get_center()}')
# ===========================================================

# -------------------------- 可视化 --------------------------
#o3d.visualization.draw_geometries([pcd, pcd_tx, pcd_new])
# ===========================================================

print("\n->采用欧拉角进行点云旋转")
pcd_EulerAngle = copy.deepcopy(pcd_new)
R1 = pcd.get_rotation_matrix_from_xyz((np.pi/2,-np.pi/8,0))
center = pcd_new.get_center()
print("旋转矩阵：\n",R1)
pcd_EulerAngle.rotate(R1,center)    # 不指定旋转中心
pcd_ground = pcd_EulerAngle.paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle质心：",pcd_EulerAngle.get_center())
# ===========================================================

# -------------------------- 可视化 --------------------------
o3d.visualization.draw_geometries([pcd, pcd_EulerAngle])
o3d.io.write_point_cloud("ground_regis.pcd",pcd_EulerAngle)
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 18:55:36 2023

@author: yangx
"""
import copy  # 点云深拷贝
import open3d as o3d
import numpy as np

input_source="ground_source.pcd"
input_target="ground.pcd"
#input_target2="right.pcd"
    
pcd_source = o3d.io.read_point_cloud(input_source)
a = pcd_source.get_center()
aabb = pcd_source.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
#aabb_center = aabb.get_center()
obb = pcd_source.get_oriented_bounding_box()
obb.color = (0, 1, 0)

print(pcd_source)
print(f'pcd质心:：{a}')
#print(f'pcd_bounding_box_center:：{aabb_center}')

pcd_tar = o3d.io.read_point_cloud(input_target)
pcd_target = copy.deepcopy(pcd_tar)
pcd_new = pcd_target.translate(a,relative=False)
pcd_new.paint_uniform_color([1, 0, 0])
print(pcd_new)

# pcd_tar2 = o3d.io.read_point_cloud(input_target2)
# pcd_target2 = copy.deepcopy(pcd_tar2)
# pcd_target2.get_center()
# b = (a[0],a[1]+20,a[2])
# pcd_new2 = pcd_target2.translate(b,relative=False)
# pcd_new2.paint_uniform_color([1, 0, 0])
# print(pcd_new2)

print(f'pcd_new质心：{pcd_new.get_center()}')
#print(f'pcd_new2质心：{pcd_new2.get_center()}')
o3d.visualization.draw_geometries([pcd_source, pcd_new, aabb, obb])

#============================================
#Rotation
#============================================
print("\n->采用欧拉角进行点云旋转")
pcd_EulerAngle = copy.deepcopy(pcd_new)
R1 = pcd_new.get_rotation_matrix_from_xyz((np.pi/2,0,0))
center = pcd_new.get_center()
print("旋转矩阵：\n",R1)
pcd_EulerAngle.rotate(R1,center)    # 不指定旋转中心
pcd_ground = pcd_EulerAngle.paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle质心：",pcd_EulerAngle.get_center())

# print("\n->采用欧拉角进行点云旋转")
# pcd_EulerAngle2 = copy.deepcopy(pcd_new2)
# R2 = pcd_new2.get_rotation_matrix_from_xyz((np.pi/2,-np.pi/6,0))
# center2 = pcd_new2.get_center()
# print("旋转矩阵：\n",R2)
# pcd_EulerAngle2.rotate(R2,center2)    # 不指定旋转中心
# pcd_ground2 = pcd_EulerAngle2.paint_uniform_color([0,0,1])
# print("\n->pcd_EulerAngle2质心：",pcd_EulerAngle2.get_center())
# ===========================================================

# -------------------------- 可视化 --------------------------
o3d.visualization.draw_geometries([pcd_source, pcd_EulerAngle])
o3d.io.write_point_cloud("ground_regis.pcd",pcd_EulerAngle)
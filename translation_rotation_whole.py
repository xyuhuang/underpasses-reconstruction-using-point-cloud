# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 13:21:59 2023

@author: yangx
"""

import open3d as o3d
import numpy as np
import copy

input_source="nonblack_2.pcd"
input_target="left_t.pcd"
input_target2="right_t.pcd"
input_target3="ground.pcd"
input_target4="roof_t.pcd"
    
pcd_source = o3d.io.read_point_cloud(input_source)
a = pcd_source.get_center()
aabb = pcd_source.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
#aabb_center = aabb.get_center()
obb = pcd_source.get_oriented_bounding_box()
obb.color = (0, 1, 0)

print(pcd_source)
print(f'pcd barycenter：{a}')
#print(f'pcd_bounding_box_center:：{aabb_center}')

#===================================================
#translation
#===================================================
pcd_tar = o3d.io.read_point_cloud(input_target)
pcd_target = copy.deepcopy(pcd_tar)
b = (a[0]-6,a[1]-9,a[2]+1)
pcd_new = pcd_target.translate(b,relative=False)
pcd_new.paint_uniform_color([1, 0, 0])
print(pcd_new)

pcd_tar2 = o3d.io.read_point_cloud(input_target2)
pcd_target2 = copy.deepcopy(pcd_tar2)
c = (a[0]+5,a[1]+7,a[2]+0.5)
pcd_new2 = pcd_target2.translate(c,relative=False)
pcd_new2.paint_uniform_color([1, 0, 0])
print(pcd_new2)

#ground
pcd_tar3 = o3d.io.read_point_cloud(input_target3)
pcd_target3 = copy.deepcopy(pcd_tar3)
d = (a[0],a[1],a[2]-2)
pcd_new3 = pcd_target3.translate(d,relative=False)
pcd_new3.paint_uniform_color([1, 0, 0])
print(pcd_new3)

#roof
pcd_tar4 = o3d.io.read_point_cloud(input_target4)
pcd_target4 = copy.deepcopy(pcd_tar4)
e = (a[0]-0.5,a[1],a[2]+1)
pcd_new4 = pcd_target4.translate(e,relative=False)
pcd_new4.paint_uniform_color([1, 0, 0])
print(pcd_new4)

print(f'pcd_new barycenter：{pcd_new.get_center()}')
print(f'pcd_new2 barycenter：{pcd_new2.get_center()}')
print(f'pcd_new3 barycenter：{pcd_new3.get_center()}')
print(f'pcd_new4 barycenter：{pcd_new4.get_center()}')
o3d.visualization.draw_geometries([pcd_source, pcd_new, pcd_new2, pcd_new3, pcd_new4])

#============================================
#Rotation
#============================================
#wall_left
print("\n->Point cloud rotation by Euler angles")
pcd_EulerAngle = copy.deepcopy(pcd_new)
R1 = pcd_new.get_rotation_matrix_from_xyz((np.pi/2,-np.pi/8,0))
center = pcd_new.get_center()
print("Rotation matrix1：\n",R1)
pcd_EulerAngle.rotate(R1,center)    # Specifying the centre of rotation
b2 = (center[0]+1,center[1],center[2])
pcd_ground = pcd_EulerAngle.translate(b2,relative=False)
pcd_ground = pcd_ground .paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle barycenter：",pcd_ground.get_center())

#wall_right
print("\n->Point cloud rotation by Euler angles")
pcd_EulerAngle2 = copy.deepcopy(pcd_new2)
R2 = pcd_new2.get_rotation_matrix_from_xyz((np.pi/2,-np.pi/9,0))
center2 = pcd_new2.get_center()
print("Rotation matrix2：\n",R2)
pcd_EulerAngle2.rotate(R2,center2)    # Specifying the centre of rotation
c2 = (center2[0]+0.1,center2[1],center2[2])
pcd_ground2 = pcd_EulerAngle2.translate(c2,relative=False)
pcd_ground2 = pcd_ground2.paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle2 barycenter：",pcd_ground2.get_center())

#ground
print("\n->Point cloud rotation by Euler angles")
pcd_EulerAngle3 = copy.deepcopy(pcd_new3)
R3 = pcd_new3.get_rotation_matrix_from_xyz((np.pi/2-np.pi/200,-np.pi/8,0))
center3 = pcd_new3.get_center()
print("Rotation matrix3：\n",R3)
pcd_EulerAngle3.rotate(R3,center3)    # Specifying the centre of rotation
d2 = (center3[0]-0.3,center3[1]-1,center3[2])
pcd_ground3 = pcd_EulerAngle3.translate(d2,relative=False)
pcd_ground3 = pcd_ground3.paint_uniform_color([0,0,1])
pcd_ground3 = pcd_EulerAngle3.paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle3 barycenter：",pcd_EulerAngle3.get_center())

#roof
print("\n->Point cloud rotation by Euler angles")
pcd_EulerAngle4 = copy.deepcopy(pcd_new4)
R4 = pcd_new4.get_rotation_matrix_from_xyz((np.pi/2-np.pi/230,-np.pi/8+np.pi/100,0))
center4 = pcd_new4.get_center()
print("Rotation matrix4：\n",R4)
pcd_EulerAngle4.rotate(R4,center4)    # Specifying the centre of rotation
e2 = (center4[0]-1,center4[1],center4[2]+2.8)
pcd_ground4 = pcd_EulerAngle4.translate(e2,relative=False)
pcd_ground4 = pcd_ground4.paint_uniform_color([0,0,1])
pcd_ground4 = pcd_EulerAngle4.paint_uniform_color([0,0,1])
print("\n->pcd_EulerAngle4 barycenter：",pcd_EulerAngle4.get_center())
# ===========================================================

# -------------------------- Visualisation --------------------------
#pcd_combined = o3d.geometry.PointCloud()
#pcds = load_point_clouds([pcd_EulerAngle,pcd_EulerAngle2])

# ---------------Stitching two point clouds together------------------
pcd_connect = o3d.geometry.PointCloud()
pcd_connect.points = o3d.utility.Vector3dVector(np.concatenate((np.asarray(pcd_ground.points),
                                                                np.asarray(pcd_ground2.points),
                                                                np.asarray(pcd_ground3.points),
                                                                np.asarray(pcd_ground4.points))))  # Splicing point cloud


o3d.visualization.draw_geometries([pcd_ground,pcd_ground2,pcd_ground3,pcd_ground4])
# o3d.io.write_point_cloud("whole_wall_left_regis.pcd",pcd_ground)
# o3d.io.write_point_cloud("whole_wall_right_regis.pcd",pcd_ground2)
# o3d.io.write_point_cloud("whole_ground_regis.pcd",pcd_ground3)
# o3d.io.write_point_cloud("whole_roof_regis.pcd",pcd_ground4)
# o3d.io.write_point_cloud("whole_regis.pcd",pcd_connect)
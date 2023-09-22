# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 15:36:38 2023

@author: 93982
"""

import open3d as o3d
import numpy as np
import trimesh
import matplotlib.pyplot as plt

#import open3d_tutorial as o3dtut

#mesh = o3dtut.get_bunny_mesh()

pcd = o3d.io.read_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.pcd")
#o3d.visualization.draw_geometries([pcd])

###########Alpha shapes
alpha = 2.4
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        pcd, alpha, tetra_mesh, pt_map)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

############Ball pivoting

# gt_mesh = o3d.io.read_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.pcd")
# gt_mesh.compute_vertex_normals()
# pcd = gt_mesh.sample_points_poisson_disk(3000)
# o3d.visualization.draw_geometries([pcd])

# radii = [0.005, 0.01, 0.02, 0.04]
# rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     pcd, o3d.utility.DoubleVector(radii))
# o3d.visualization.draw_geometries([pcd, rec_mesh])


############Normal Distribution
# gt_mesh = o3d.io.read_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.pcd")
# pcd = gt_mesh.sample_points_poisson_disk(5000)
# pcd.normals = o3d.utility.Vector3dVector(np.zeros(
#     (1, 3)))  # invalidate existing normals

# pcd.estimate_normals()
# o3d.visualization.draw_geometries([pcd], point_show_normal=True)

# pcd.orient_normals_consistent_tangent_plane(100)
# o3d.visualization.draw_geometries([pcd], point_show_normal=True)

###########convex hull
# hull, idx = pcd.compute_convex_hull()
# hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull) # 生成三角网
# hull_ls.paint_uniform_color((1, 0, 0))  
# o3d.visualization.draw_geometries([pcd, hull_ls]) # 同时显示两组点云
# # hull, idx = pcd.compute_convex_hull()
# hull_cloud = pcd.select_by_index(idx)
# hull_cloud.paint_uniform_color((0, 0, 1))  
# o3d.visualization.draw_geometries([hull_ls,hull_cloud, pcd])

############Poisson

# input_ground="./training/labelledLocalCRS/DEBY_LOD2_4959462/whole_regis.pcd"
    
# target = o3d.io.read_point_cloud(input_ground)
# #source = o3d.io.read_point_cloud('nonblack_2.pcd')
# o3d.visualization.draw_geometries([target])
# #downpcd = target.voxel_down_sample(voxel_size=0.05)
# target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# # o3d.visualization.draw_geometries([downpcd],
# #                                    zoom=0.3412,
# #                                    front=[0.4257, -0.2125, -0.8795],
# #                                    lookat=[2.6172, 2.0475, 1.532],
# #                                    up=[-0.0694, -0.9768, 0.2024],
# #                                    point_show_normal=True)
# #===============================

# #Poisson
# print('run Poisson surface reconstruction')
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(target, depth=12)
# print(mesh)
# o3d.visualization.draw_geometries([mesh], zoom=0.664,
#                                   front=[-0.4761, -0.4698, -0.7434],
#                                   lookat=[1.8900, 3.2596, 0.9284],
#                                   up=[0.2304, -0.8825, 0.4101])

# print('visualize densities')
# densities = np.asarray(densities)
# density_colors = plt.get_cmap('plasma')(
#     (densities - densities.min()) / (densities.max() - densities.min()))
# density_colors = density_colors[:, :3]
# density_mesh = o3d.geometry.TriangleMesh()
# density_mesh.vertices = mesh.vertices
# density_mesh.triangles = mesh.triangles
# density_mesh.triangle_normals = mesh.triangle_normals
# density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
# o3d.visualization.draw_geometries([density_mesh], zoom=0.664,
#                                   front=[-0.4761, -0.4698, -0.7434],
#                                   lookat=[1.8900, 3.2596, 0.9284],
#                                   up=[0.2304, -0.8825, 0.4101])

# print('remove low density vertices')
# vertices_to_remove = densities < np.quantile(densities, 0.02)
# mesh.remove_vertices_by_mask(vertices_to_remove)
# print(mesh)
 
 
# o3d.visualization.draw_geometries([mesh], zoom=0.664,
#                                     front=[-0.4761, -0.4698, -0.7434],
#                                     lookat=[1.8900, 3.2596, 0.9284],
#                                     up=[0.2304, -0.8825, 0.4101])



##############voxel
# fit to unit cube

# print(pcd)
# N = np.asarray(pcd.points).shape[0]
# pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
# # From numpy to Open3D
# pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0,1,size=(N,3)))
# o3d.visualization.draw_geometries([pcd])

# # 体素化
# print('voxelization')
# voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.01)
# # 可视化
# o3d.visualization.draw_geometries([voxel_grid])
# # 同时显示点和体素
# #o3d.visualization.draw_geometries([pcd,voxel_grid],mesh_show_wireframe = True)
# #o3d.io.write_point_cloud("./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_whole.ply", voxel_grid)
# voxel_grid.export("./training/labelledLocalCRS/DEBY_LOD2_4959462/voxel_whole.ply", file_type='ply')
# # 判断点是否占用体素
# queries = np.asarray(pcd.points)
# output = voxel_grid.check_if_included(o3d.utility.Vector3dVector(queries))
# print(output[:20])

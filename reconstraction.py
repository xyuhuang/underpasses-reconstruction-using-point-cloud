# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 23:09:04 2023

@author: yangx
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


input_ground="whole_regis.pcd"

    
target = o3d.io.read_point_cloud(input_ground)
o3d.visualization.draw_geometries([target])
target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# o3d.visualization.draw_geometries([downpcd],
#                                    zoom=0.3412,
#                                    front=[0.4257, -0.2125, -0.8795],
#                                    lookat=[2.6172, 2.0475, 1.532],
#                                    up=[-0.0694, -0.9768, 0.2024],
#                                    point_show_normal=True)
#===============================


#==================================================
#Ball Pivoting Algorithm - BPA
#==================================================
distances = target.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 5 * avg_dist
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(target,o3d.utility.DoubleVector([radius, radius * 2]))
dec_mesh = bpa_mesh.simplify_quadric_decimation(300000)
dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()
o3d.visualization.draw_geometries([dec_mesh])
#==================================================
 


#==================================================
#Poisson
#==================================================
print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(target, depth=12)
print(mesh)
o3d.visualization.draw_geometries([mesh], zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])

print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh], zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.02)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
 
 
o3d.visualization.draw_geometries([mesh], zoom=0.664,
                                    front=[-0.4761, -0.4698, -0.7434],
                                    lookat=[1.8900, 3.2596, 0.9284],
                                    up=[0.2304, -0.8825, 0.4101])
o3d.io.write_triangle_mesh("underpass.obj", mesh)
#=====================================================




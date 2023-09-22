# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 16:34:47 2023

@author: 93982
"""

import trimesh

def to_ply(input_path, output_path, original_type):
    mesh = trimesh.load(input_path, file_type=original_type)  # read file
    mesh.export(output_path, file_type='ply')  # convert to ply
 

input_path='./training/labelledLocalCRS/DEBY_LOD2_4959462/ground.obj'
output_path='./training/labelledLocalCRS/DEBY_LOD2_4959462/ground.ply' 
    
to_ply(input_path, output_path, 'obj')
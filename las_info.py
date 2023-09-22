# -*- coding: utf-8 -*-

import read_pcl
from pclpy import pcl
#import matplotlib.pyplot as plt
import time
from laspy.file import File
import laspy
import numpy as np
import os
import functools

def log_execution_time(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        res = func(*args, **kwargs)
        end = time.perf_counter()
        print('【%s】 took %.2f s' % (func.__name__, (end - start)))
        return res
    return wrapper


def lasinfo():
    #f = File(path, mode='r')
    f = laspy.read(path) #18*18581803
    print('datatype:',type(f))
    #print('size of file: ', np.shape(f))
    # 查看点云的点格式及字段名称
    #print('\nPoint Of Data Format: ', f.header.data_format_id)
    print("\tExamining Point Format: ", end=" ")
    for spec in f.point_format:
        print(spec.name, end=", ")
    
    print('\nlength of file:',len(f))
    print('\noffset: ', f.header.offset)  # 偏移量
    print('scale: ', f.header.scale)  # 比例因子
    print('min: ', f.header.min)  # x、y、z 的最小值
    print('max: ', f.header.max)  # x、y、z 的最大值
    #print('file_signature: ', f.header.file_signature)  # 文件标识
    #print('Point Of Data Format: ', f.header.data_format_id)  # 点格式
    #print('data_record_length: ', f.header.data_record_length)  # 点个数
    print('FileCreateDay+Year: ', f.header.date)
    print()
    print('f.x: ', f.x)
    print('f.y: ', f.y)
    print('f.z: ', f.z)
    print('f.intensity: ', f.intensity)
    #print('f.gps_time: ', f.gps_time)
    print('f.raw_classification: ', f.raw_classification)
    print()
    return f


##################################
path = "D:/TUM File/semester3/photogrammetry project/data/training/labelledLocalCRS/DEBY_LOD2_4959462/DEBY_LOD2_4959462.las"

f=lasinfo()



print ('end')


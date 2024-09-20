# -*- coding: utf-8 -*-
# encoding: utf-8print
import pyrealsense2 as rs
import numpy as np
import cv2

'''
开启点云
'''
# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

'''
设置
'''
 # 定义流程pipeline，创建一个管道
pipeline = rs.pipeline()

# 定义配置config
config = rs.config()
# 颜色和深度流的不同分辨率
# 配置depth流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
# config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
# config.enable_stream(rs.stream.depth,  1280, 720, rs.format.z16, 30)

# 配置color流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# streaming流开始
pipe_profile = pipeline.start(config)

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
# 获取深度传感器的深度标尺（参见rs - align示例进行说明）
depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)      # ('Depth Scale is: ', 0.0010000000474974513)


# 创建对齐对象与color流对齐
# align_to 是计划对齐深度帧的流类型
align_to = rs.stream.color
# rs.align 执行深度帧与其他帧的对齐
align = rs.align(align_to)
count=1
 # Streaming循环
while True:
    '''
    获取图像帧与相机参数
    '''
    # 等待获取图像帧，获取颜色和深度的框架集
    frames = pipeline.wait_for_frames()         # frames.get_depth_frame（）是640x360深度图像
    # 获取对齐帧，将深度框与颜色框对齐
    aligned_frames = align.process(frames)
    # 获取对齐帧中的的depth帧
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame是640x480深度图像
    # 获取对齐帧中的的color帧
    aligned_color_frame = aligned_frames.get_color_frame()

    # 将images转为numpy arrays
    # RGB图
    img_color = np.asanyarray(aligned_color_frame.get_data())
    # 深度图（默认16位）
    img_depth = np.asanyarray(aligned_depth_frame.get_data())

    # Intrinsics & Extrinsics
    # 获取深度参数（像素坐标系转相机坐标系会用到）
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    # 获取相机内参
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
    # 获取两摄像头之间的外参
    depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(aligned_color_frame.profile)

    # 设置测试随机点
    # Map depth to color
    # depth_pixel = [320, 240]   # Random pixel
    # depth_pixel2 = [240, 240]   # Random pixel

    # x = depth_pixel[0]
    # y = depth_pixel[1]

    # x2 = depth_pixel2[0]
    # y2 = depth_pixel2[1]


    # '''
    # 方法一：获取三维坐标(rs2_deproject_pixel_to_point方法)
    # '''
    # # rs2_deproject_pixel_to_point方法，2d转3d，获得三维坐标
    # # camera_coordinate = rs.rs2_deproject_pixel_to_point(intrin=depth_intrin, pixel=[x, y], depth=dis)
    # # depth_intrin 从上一步获取
    # # x 像素点的x
    # # y 像素点的y
    # # dis 上一步计算的真实距离（输入的dis与输出的距离是一样的，改变的只是x与y
    dis = aligned_depth_frame.get_distance(x, y)         # 读取深度图中目标点的深度信息，单位是m
    dis2 = aligned_depth_frame.get_distance(x2, y2)         # 读取深度图中目标点的深度信息，单位是m

    # print ('===============方法1：二维映射三维函数=============')
    print ('depth: ',dis)       # ('depth: ', 2.502000093460083)
    print ('depth2: ',dis2)       # ('depth: ', 2.502000093460083)

    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    #print ('camera_coordinate: ',camera_coordinate)     # ('camera_coordinate: ', [-0.022640999406576157, -0.03151676058769226, 2.5230000019073486])

    color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, camera_coordinate)
    color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
    # print ('color_point: ',color_point)     # ('color_point: ', [-0.022640999406576157, -0.03151676058769226, 2.5230000019073486])
    # print ('color_pixel: ',color_pixel)     # ('color_pixel: ', [320.0, 240.0])


    '''
    方法二：获取三维坐标（点云的另一种计算方法）
    '''
    # print ('===============方法2：点云=============')
    # pc = rs.pointcloud()
    # frames = pipeline.wait_for_frames()
    # depth = frames.get_depth_frame()
    # color = frames.get_color_frame()
    # img_color = np.asanyarray(color_frame.get_data())
    # img_depth = np.asanyarray(depth_frame.get_data())
    pc.map_to(aligned_color_frame)
    points = pc.calculate(aligned_depth_frame)
    vtx = np.asanyarray(points.get_vertices())
    # tex = np.asanyarray(points.get_texture_coordinates())

    npy_vtx = np.zeros((len(vtx), 3), float)
    for i in range(len(vtx)):
        npy_vtx[i][0] = np.float(vtx[i][0])
        npy_vtx[i][1] = np.float(vtx[i][1])
        npy_vtx[i][2] = np.float(vtx[i][2])
        
    #得到npy_vtx：存储了所有点云的三维坐标（未乘旋转矩阵)
    matrix_30deg = np.transpose(npy_vtx)
    angle = np.pi/3-0.175
    
    rotation_matrix = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])

    matrix_upright = np.matmul(rotation_matrix, matrix_30deg)

    print("Shape of matrix_upright: ",matrix_upright.shape) # 3*32500
    print(matrix_upright)
    print(count)
    count+=1
    
    # for coordinate in matrix_upright:
    #     if coordinate[0]>-0.058 and coordinate[0]<0:

"""
以下为使用官方计算
"""

import pyrealsense2 as rs
import numpy as np
import os, datetime, time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# 设置深度推流
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
# 设置近红外推流
config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 15)
config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 15)
# 设置RGB推流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
# Start streaming


 

# 创建对齐对象（深度对齐颜色）
while True:
    ymd = datetime.datetime.now().strftime('%Y-%m-%d')
    ymd_path = os.path.join('data', ymd)
    if not os.path.exists(ymd_path):
        os.makedirs(ymd_path)

    hm = datetime.datetime.now().strftime('%H-%M')
    hm_path = os.path.join(ymd_path, hm)
    if not os.path.exists(hm_path):
        os.makedirs(hm_path)

    subfolders = ['rgb', 'depth', 'if_left', 'if_right']
    for folder in subfolders:
        folder_path = os.path.join(hm_path, folder)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
    pipeline.start(config)
    time.sleep(30)
    try:
        # Wait for a coherent pair of frames: depth and color
        for i in range(10):
            print(i)
            # 对齐后再获取
            for j in range(15):
                frames = pipeline.wait_for_frames()
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if1_frame = frames.get_infrared_frame(1)
            if2_frame = frames.get_infrared_frame(2)
            
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            if1_image = np.asanyarray(if1_frame.get_data())
            if2_image = np.asanyarray(if2_frame.get_data())
            
            np.save(os.path.join(hm_path, 'rgb', '%d.npy'%i ),color_image)
            np.save(os.path.join(hm_path, 'depth', '%d.npy'%i),depth_image)
            np.save(os.path.join(hm_path, 'if_left', '%d.npy'%i),if1_image)
            np.save(os.path.join(hm_path, 'if_right', '%d.npy'%i),if2_image)
            #colorizer.colorize(depth_frame)
        #pc.map_to(color_frame)
        #points = pc.calculate(depth_frame)
        #points.export_to_ply('./outnew.pcd', color_frame)
    finally:
        pipeline.stop()
        time.sleep(30)
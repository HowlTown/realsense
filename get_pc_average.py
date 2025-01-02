"""
以下为使用官方计算
"""

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import gc

def depth_and_color_to_pointcloud(depth_image, color_image, intrinsic):
    # Create Open3D RGBD Image 
    o3d_depth = o3d.geometry.Image(depth_image)
    o3d_color = o3d.geometry.Image(color_image)
    o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)
    
    # Get intrinsic parameters
    fx, fy, cx, cy = intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy

    # Create Open3D PinholeCameraIntrinsic object
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width = depth_image.shape[1], 
        height = depth_image.shape[0], 
        fx = intrinsic.fx, 
        fy = intrinsic.fy,
        cx = intrinsic.ppx, 
        cy = intrinsic.ppy)

    # Create Open3D PointCloud object from depth image and intrinsic parameters
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, o3d_intrinsic)

    return pcd

# Configure depth and color streams
pipeline = rs.pipeline()
print(111)
config = rs.config()

# 设置深度推流
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
# 设置RGB推流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
# Start streaming
profile = pipeline.start(config)
print(222)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# 创建对齐对象（深度对齐颜色）
align = rs.align(rs.stream.color)
pc = rs.pointcloud()
colorizer = rs.colorizer()
print(333)
try:
    # Wait for a coherent pair of frames: depth and color
    pcd_list = []
    depth_image_list = []
    color_image_list = []
    for i in range(10):
        print(i)
        for j in range(14):
            frames = pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        # 对齐后再获取
        aligned_frames = align.process(frames)
        # 深度信息
        depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        #print(i,depth_intrinsics)
        # rgb信息
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image_list.append(depth_image)
        color_image_list.append(color_image)
        #pcd = depth_and_color_to_pointcloud(depth_image, color_image, depth_intrinsics)
        #pcd_list.append(pcd)
    depth_image = np.median(np.stack(depth_image_list, axis=0), axis=0).astype(np.uint16)
    color_image = np.median(np.stack(color_image_list, axis=0), axis=0).astype(np.uint8)
    print(444)
    pcd = depth_and_color_to_pointcloud(depth_image, color_image, depth_intrinsics)
    o3d.io.write_point_cloud("outnew.pcd", pcd)
    
finally:
    pipeline.stop()
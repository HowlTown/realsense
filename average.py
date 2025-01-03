import numpy as np
import open3d as o3d
import os
from show import show_pointcloud

class intrinsics():
    def __init__(self,fx,fy,ppx,ppy):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy
        

date = "2024-12-27"
time = "20-04"
folders = ["depth","rgb"]


class RealSenseImages():
    def __init__(self):
        self.intrinsics = intrinsics(0,0,0,0)
        self.raw = {}
        self.depth_image = []
        self.color_image = []
        self.pcd = None
        
    def set_intrinsics(self,i):
        self.intrinsics = i
    
    def load_images(self,date,time):
        folders = ["depth","rgb"]
        for folder in folders:
            self.raw[folder] = []
            folder_path = os.path.join("data",date,time,folder)
            for i in range(10):
                image_path = os.path.join(folder_path,"%d.npy" % i)
                image = np.load(image_path)
                self.raw[folder].append(image)
                
    def calculate_pointclouds(self):
        o3d_depth = o3d.geometry.Image(self.depth_image.astype(np.uint16))
        o3d_color = o3d.geometry.Image(self.color_image.astype(np.uint8))
        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)
        
        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = self.depth_image.shape[1], 
            height = self.depth_image.shape[0], 
            fx = self.intrinsics.fx, 
            fy = self.intrinsics.fy,
            cx = self.intrinsics.ppx, 
            cy = self.intrinsics.ppy)

        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, o3d_intrinsic)
        o3d.io.write_point_cloud("outnew.pcd", self.pcd)
        
    
    def fliter_images(self):
        height, width = self.raw["depth"][0].shape[0],self.raw["depth"][0].shape[1]
        self.depth_image = np.zeros((height,width))
        self.color_image = np.zeros((height,width))
        for y in range(height):
            for x in range(width):
                depth_list = [depth[y,x] for depth in self.raw["depth"]]
                if np.max(depth_list) - np.min(depth_list) < np.median(depth_list):
                    self.depth_image[y,x] = np.median(depth_list)
                    self.color_image[y,x] = np.median(
                        [color[y,x] for color in self.raw["rgb"]]
                    )
                
if __name__ == "__main__":
    depth_intrinsics = intrinsics(652.8689575195312,652.8689575195312,651.2142944335938,363.8994140625)
    rsi = RealSenseImages()
    rsi.set_intrinsics(depth_intrinsics)
    rsi.load_images("2024-12-27","20-04")
    rsi.fliter_images()
    rsi.calculate_pointclouds()
    
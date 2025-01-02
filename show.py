import open3d as o3d
import numpy as np

def show_pointcloud(file,max_distance):
    pcd = o3d.io.read_point_cloud(file)
    distances = np.linalg.norm(np.asarray(pcd.points), axis=1)
    # 过滤掉距离超过 1 米的点
    mask = distances <= 1.0
    pcd_filtered = pcd.select_by_index(np.where(mask)[0])
    # 可视化过滤后的点云
    o3d.visualization.draw_geometries([pcd_filtered])


import open3d as o3d
import numpy as np

# 读取PLY文件
#pcd = o3d.io.read_point_cloud('data_0912\point_clouds\point_cloud_20230912_0000.pcd')
#pcd = o3d.io.read_point_cloud('1.ply')
pcd = o3d.io.read_point_cloud('outnew.pcd')

# 翻转点云，使其正确显示
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


# 计算每个点到原点的距离
distances = np.linalg.norm(np.asarray(pcd.points), axis=1)

# 过滤掉距离超过 1 米的点
mask = distances <= 1.0
pcd_filtered = pcd.select_by_index(np.where(mask)[0])

# 可视化过滤后的点云
o3d.visualization.draw_geometries([pcd_filtered])

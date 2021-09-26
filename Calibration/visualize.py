import numpy as np
import open3d as o3d

# How to import open3D
# http://www.open3d.org/docs/release/getting_started.html#installing-from-pypi-or-conda


pcd = o3d.io.read_point_cloud("/Users/maido/Documents/Matlab/lidar_example.pcd")

print(pcd)

print(np.asarray(pcd.points))

o3d.visualization.draw_geometries([pcd])
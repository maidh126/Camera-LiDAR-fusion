import numpy as np
import open3d as o3d
import math

### Takes a path to a .pcd file and returns an array of 3D points [x,y,z]
def pcdToArray(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    # return np.asarray(pcd.points)
    return pcd

### Takes in the array of points and saves it into a .pcd file format
def arrayToPcd(pcd_path, pcd):
    cloud = np.asarray(pcd.points)
    with open(pcd_path, "w") as f:
        header = "{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n".format(
            "VERSION .7",
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            "WIDTH " + str(cloud.shape[0]),
            "HEIGHT 1",
            "VIDEPOINT 0 0 0 1 0 0 0",
            "POINTS " + str(cloud.shape[0]),
            "DATA ascii"
        )
        f.write(header)

        for r in range(cloud.shape[0]):
            point = "{} {} {} {}\n".format(cloud[r,0], cloud[r,1], cloud[r,2], cloud[r,3])
            f.write(point)
    return None

### Takes a FOV angle (degrees) and a pointcloud (points array) to crop
### Returns a cropped pointcloud (array) 
def cropFOV(angle_start, angle_finish, cloud):
    pcd = o3d.geometry.PointCloud()
    new_points = list()
    for point in cloud.points:
        # calculate the angle at which the point is located
        # to do this, we need to know the x and y coordinated of the point
        angle = math.degrees(np.arctan2(point[1], point[0]))
        # if the point lies within the required range of angles (e.g. -45, +45 degrees), keep the point
        if angle >= angle_start and angle <= angle_finish:
            new_points.append(point)

    pcd.points = o3d.utility.Vector3dVector(np.array(new_points, dtype=np.float64))
    return pcd

if __name__ == "__main__":
    #pcd = cropFOV(-40, 27, pcdToArray("/Users/maido/Documents/Matlab/lidar_example3.pcd"))
    pcd = cropFOV(-40, 27, pcdToArray("/Users/maido/Documents/Matlab/lidar_example.pcd"))
    #o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud("lidar_example11.pcd", pcd)
    #arrayToPcd("/Users/maido/Documents/Matlab/lidar_example2.pcd", pcd)
    new_pcd = o3d.io.read_point_cloud("lidar_example11.pcd")
    o3d.visualization.draw_geometries([new_pcd])

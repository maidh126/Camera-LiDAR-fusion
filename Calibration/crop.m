
% https://uk.mathworks.com/help/vision/ref/pointcloud.findpointsinroi.html

% Read point clouds

zed = pcread("/Users/maido/Documents/Matlab/zed_example.pcd");

lidar = pcread("/Users/maido/Documents/Matlab/lidar_example.pcd");

% Show point cloud

pcshow(zed.Location);
pcshow(lidar.Location);


% Define a cuboid ROI within the range of the x, y and z coordinates of the input point cloud.

roi = [-2 2 -2 2 2.4 3.5];


% Find the indices of the points that lie within the cuboid ROI.

indices = findPointsInROI(ptCloud,roi);



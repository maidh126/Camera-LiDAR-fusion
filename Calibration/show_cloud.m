% ptCloud = pcread("calibration pcd/0278(0m25sec).pcd");
lidarCloud = pcread("/Users/maido/Documents/Matlab/lidar_example.pcd");
zedCloud = pcread("/Users/maido/Documents/Matlab/zed_example.pcd");

%pcshow(ptCloud.Location)
pcshowpair(zedCloud,lidarCloud)
xlabel('X')
ylabel('Y')
zlabel('Z');

%load('lidar_example.pcd')
%ptCloud = pointCloud(zedCloudCrop)
%ptCloud


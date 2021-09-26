ptCloud = pcread("/Users/maido/Documents/Matlab/zed_example.pcd");
boardCloud = pcread("/Users/maido/Documents/Matlab/lidar_example.pcd");
pcshowpair(ptCloud,boardCloud)
title('Detected Rectangular Plane')
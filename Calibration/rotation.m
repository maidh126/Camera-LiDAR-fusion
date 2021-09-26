
ptCloud = pcread("/Users/maido/Documents/Matlab/zed_example.pcd");

boardCloud = pcread("/Users/maido/Documents/OBU/Thesis/LIDAR/lidar_example11.pcd");
%boardCloud = pcread("/Users/maido/Documents/Matlab/lidar_example.pcd");


% Create a transform object with a 180 degree rotation along the z-axis. (upside down)

rot = [cos(pi) -sin(pi) 0; ...
      sin(pi)  cos(pi)  0; ...
      0          0      1];

  
trans = [0, 0, 0];
tform = rigid3d(rot,trans);

% Transform the point cloud.
ptCloudOut = pctransform(ptCloud,tform);



rot2 = [cos(pi/2) -sin(pi/2) 0; ...
      sin(pi/2)  cos(pi/2)  0; ...
      0          0      1];

tform2 = rigid3d(rot2,trans);
ptCloudOut2 = pctransform(ptCloudOut,tform2);




rot3 = [cos(pi/2) 0 sin(pi/2); ...
       0            1 0; ...
       -sin(pi/2) 0 cos(pi/2)];

   
   
trans2 = [0.18, -0.1, -0.05];   
tform3 = rigid3d(rot3,trans2);
ptCloudOut3 = pctransform(ptCloudOut2,tform3);



%reflection
refl = [1 0 0 0; ...
        0 -1 0 0; ...
        0 0 1 0; ...
        0 0 0 1];
tform4 = affine3d(refl);
    
ptCloudOut4 = pctransform(ptCloudOut3,tform4);


refl2 = [1 0 0 0; ...
        0 1 0 0; ...
        0 0 -1 0; ...
        0 0 0 1];
tform5 = affine3d(refl2);
    
zedCloud = pctransform(ptCloudOut4,tform5);



% for lidar
rot_ld = [cos(-pi/27) -sin(-pi/27) 0; ...
          sin(-pi/27)  cos(-pi/27)  0; ...
          0          0      1];

  
trans_ld = [0, 0, 0];
tform_ld = rigid3d(rot_ld,trans_ld);

% Transform the lidar point cloud.
lidarCloud = pctransform(boardCloud,tform_ld);


pcshowpair(zedCloud,lidarCloud)
% title('Detected Rectangular Plane')
xlabel('X')
ylabel('Y')
zlabel('Z')



%%% CROP %%%
% https://uk.mathworks.com/help/vision/ref/pointcloud.findpointsinroi.html
% https://uk.mathworks.com/help/vision/point-cloud-processing.html

% Define a cuboid ROI within the range of the x, y and z coordinates of the input point cloud.

%roi1 = [0 50 -50 50 -2 0]; % example 2: far 50m, width 50+50m, lower 2m, no higher than lidar
roi1 = [0 8 -10 10 -2 0]; % example 1: far 8m, width 10+10m, lower 2m, no higher than lidar
roi2 = [0 8 -10 10 -2 0]; % example 1: zed

% Find the indices of the points that lie within the cuboid ROI.

indices = findPointsInROI(lidarCloud,roi1);
indices2 = findPointsInROI(zedCloud,roi2);


% Select the points that lie within the cuboid ROI and store as a point cloud object.

lidarCloudCrop = select(lidarCloud,indices);
zedCloudCrop = select(zedCloud,indices2);


pcshowpair(zedCloud,lidarCloud) % example 2
%pcshow(zedCloudCrop.Location) % example 1
%pcshow(lidarCloudCrop.Location)
%pcshowpair(zedCloudCrop,lidarCloudCrop) % example 1
% title('Detected Rectangular Plane')
xlabel('X')
ylabel('Y')
zlabel('Z')



%lidarCloudCrop



#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

LidarPoint   {x,y,z in [m], r is point reflectivity}
BoundingBox {
    boxID        :bounding box around a classified object (contains both 2D and 3D data)
    trackID      :unique identifier for the track to which this bounding box belongs
    roi          :cv.Rect 2D region-of-interest in image coordinates
    classID      :ID based on class file provided to YOLO framework
    confidence   :classification trust
            }
DataFrame {
    cameraImg    :color camera image
    keypoints    :2D keypoints within camera image
    descriptors  :keypoint descriptors
    kptMatches   :keypoint matches between previous and current frame
    lidarPoints  :
    boundingBoxes:
    bbMatches    :
          }

"""
import cv2 as cv
from recordtype import recordtype #mutable namedtuple
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from SensorProcessor import PointProcessing,CamProcessing
from YOLO import ObjectDetection,ObjectDetectionOpenCV
from utils import View, Plot

###########################################################################
#-----------------Define Data Structures and Paths------------------------#
###########################################################################       
#Define data structures
DataFrame=recordtype('DataFrame',['cameraImg','keypoints', 'descriptors' ,'kptMatches','lidarPoints','boundingBoxes','bbMatches'])
BoundingBox=recordtype('BoundingBox',['boxID','trackID','roi','classID', 'confidence', 'lidarPoints','lidarPixels','keypoints','kptMatches'])


#Data location
#Images
img_folderpath='../data/KITTI/2011_09_26/image_02/data/*.png'
img_filepaths=sorted(glob.glob(img_folderpath))
#Lidar
lidar_folderpath='../data/KITTI/2011_09_26/velodyne_points/data/*.bin'
lidar_filepaths=sorted(glob.glob(lidar_folderpath))
#Yolo Network
labelsPath = "../data/YOLO/coco.names"
weightsPath = "../data/YOLO/yolov3.weights"
configPath = "../data/YOLO/yolov3.cfg"
modelPath = "../data/YOLO/yolov3_model.h5"


#Loop over all data sequence
dataBuffer=[]
plot_results=Plot()
view=View()
LP=PointProcessing()
CP=CamProcessing()

#YOLO
params=dict()
params["confidence"]=0.2
params["threshold"]=0.4
params["shrink"]=0.1


#LiDAR
#Kitti
x_range,y_range,z_range,scale=(-20, 20),(-20, 20),(-2, 2),10
v_fov, h_fov,max_d = (-24.9, 2.0), (-90, 90),70
#Udacity
# minZ = -1.5; maxZ = -0.9; minX = 2.0; maxX = 20.0; maxY = 2.0; minR = 0.1; # focus on ego lane
minZ = -1.4; maxZ = 1e2; minX = 0.0; maxX = 25.0; maxY = 6.0; minR = 0.01;max_d=20;

        
def animate(idx):
    bBox=BoundingBox('None','None','None','None', 'None','None','None','None','None')
    frame=DataFrame('None','None', 'None' ,'None','None','None','None')
    ###########################################################################
    #-----------------------------Process Camera------------------------------#
    ###########################################################################   
    img = cv.imread(img_filepaths[idx])
    # YOLO OpenCV Implementation
    YOLO=ObjectDetectionOpenCV(params,labelsPath,weightsPath,configPath,bBox) 
    #YOLO Keras Implementation
    # YOLO=ObjectDetection(params,labelsPath,modelPath,bBox) 
    bBoxes,img_boxes=YOLO.predict(img.copy())  #returns list of bBox   

    print("#1 : LOAD IMAGE INTO BUFFER done")
    print("#2 : DETECT & CLASSIFY OBJECTS done")   
    #Compute Keypoints and Descriptprs
    img_gray=cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    kp,des=CP.featureDescript(img_gray)



    ###########################################################################
    #------------------------------Process Lidar------------------------------#
    ###########################################################################
    velodyne_scan = np.fromfile(lidar_filepaths[idx], dtype=np.float32)
    #import lidar data
    lidar_scan=velodyne_scan.reshape((-1, 4))    
    #-----------------KITTI-------------------#
    #display topview
    img_lidar_topview_kitti=view.lidar_top_view_kitti(lidar_scan, x_range, y_range, z_range, scale)#lidar_pts:nx4
    #remove lidar points outside of the field of view and compute color range based on max distance
    lidar_pts_kitti,lidar_color_kitti=LP.veloFilter(lidar_scan,v_fov,h_fov,max_d)
    print("#3 : CROP LIDAR POINTS done")
    #project lidar points to camera images
    lidar_px_kitti=LP.projectPointToImage(lidar_pts_kitti) 
    #Overlay lidar on camera image
    img_lidar_fusion_kitti=view.lidar_overlay_kitti(lidar_px_kitti, lidar_color_kitti, img.copy())      
    
    #-----------------Udacity-------------------#
    img_lidar_topview=view.lidar_top_view(lidar_scan)
    #remove Lidar points based on distance properties
    crop_range=(minZ,maxZ,minX,maxX,maxY,minR)
    lidar_pts,lidar_color=LP.cropPoint(lidar_scan,crop_range,max_d)
    #project lidar points to camera images
    lidar_px=LP.projectPointToImage(lidar_pts) 
    #Overlay lidar on camera image
    img_lidar_fusion=view.lidar_overlay(lidar_px, lidar_color, img.copy())       
    #-------------------Cluster Lidar----------------------#
    #Clustor lidar pixels with Boundingbox ROI    
    bBoxes_clustered=LP.clusterPointROI(bBoxes,lidar_px,lidar_pts)
    print( "#4 : CLUSTER LIDAR POINT CLOUD done")       
    img_clustered_lidar_topview=view.lidar_3d_objects(bBoxes_clustered)      
    img_clustered_lidar_cam_overlay=view.lidar_overlay_objects(bBoxes_clustered,img_boxes,max_d)    
    
    #Save data frame
    frame.cameraImg=img.copy()
    frame.boundingBoxes=bBoxes.copy()
    frame.keypoints=kp.copy()
    frame.descriptors=des.copy()
    frame.lidarPoints=lidar_pts.copy()    
    dataBuffer.append(frame)  


    ###########################################################################
    #-------------------------Descriptor Matching-----------------------------#
    ###########################################################################       
    if len(dataBuffer)>1:             
        last_frame=dataBuffer[-2]
        current_frame=dataBuffer[-1]
        good_matches,good_kp_last,good_kp_current=CP.matchDescript(current_frame,last_frame)        
        num_good_matches = len(good_matches)
        print("#7 : MATCH KEYPOINT DESCRIPTORS done" )  
        img_current=current_frame.cameraImg.copy()
        img_last=last_frame.cameraImg.copy()
        cv.drawKeypoints(img_last, good_kp_last,img_last,(0, 0, 255),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.drawKeypoints(img_current, good_kp_current,img_current,(0, 0, 255),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)    
        plot_results.update(img_last,img_current,img_lidar_topview ,img_lidar_fusion,img_lidar_topview_kitti)

idx_step=1
idx_stop=21#len(img_filepaths)
anim = animation.FuncAnimation(plot_results.get_figure(), animate, frames=np.arange(0,idx_stop,idx_step),repeat=False)
#plt.draw()
#plt.show()
# save animation at 21 frames per second
anim.save('SensorFusion.gif', writer='imagemagick', fps=10,dpi=200)
# How to run 

- step 1: down load  coco  weight from [here](https://github.com/sonnhfit/camera-lidar-fusion-yolo-detection-kitti/releases/download/1.0/yolov3.weights)
- step 2: put `yolov3.weights` in to `data/YOLO` folder
- step 3: install dependence with command

```
pip install -r requirements.txt
```
- step 4: generate yolo model h5 file with command
```
cd data/YOLO
python generate_yolo_model.py
```
- step 5: run 
```
python LidarCameraFusion_animate.py
```
- step 6: run 
```
python ProjectSolution_animate.py
```



# LiDAR and Camera Fusion for Estimating Time-to-Collision

![sensor_fusion](./figures/SensorFusion.gif) 


## Code Structure 

**/scripts/SensorProcessor.py :**

- CameraProcessing class implements all image processing methods
	- computeFeatureDescriptors: given two consecutive frames, computes FAST keypoint detector and BRIEF descriptors for each frame (Step 5-6)
	- matchFeatureDescriptor: computes good descriptor matches based on distance ratio test (Step 7)
	- matchBoundingBoxes: matches bounding boxes between two consecutive frames based on keypoint matches
	- clusterKptMatchesWithROI:associates a given bounding box with the keypoints it contains (Step 8)
	- computeTTCcamera:computes time-to-collision  based on keypoint correspondences in successive images (Step 9)
	
- LidarProcessing class implements all LiDAR processing methods
	- projectLidarToCam: projects 3D lidar points to camera image and returns 2D lidar coordinates in camera space 
	- velo_points_filter_kitti:crops lidar points based on [2] vertical and horizontal field of view  (pitch, yaw). (Step 3)
	- crop_lidar_points: crops lidar points based on a given 3D position range and reflectivity. (Step 3)
	- cluster_lidar_with_ROI: given anchor box coordinates from YOLO, clusters the lidar points within the bounding boxes and removes overlaps (Step 4)
	- computeTTCLidar: computes mean distance to the car in the ego lane to calculate time-to-collision (Step 9)

**/scripts/YOLO.py :**

- ObjectDetectionOpenCV class implements YOLO v3 using OpenCV dnn library [3]
	- predict: decodes YOLO network output to anchor box coordinates, labels and confidence scores and implements non-maximum suppression. (Step 2)
- ObjectDetection class implements YOLO v3 using Keras [4]
	- predict: decodes YOLO network output to anchor box coordinates, labels and confidence scores and implements non-maximum suppression. (Step 2)
- Helper functions from [4] for ObjectDetection class

**/scripts/LidarCameraFusion.py** (Steps 1-7)

**/scripts/Project_Solution.py** (Steps 8-9)

**/scripts/LidarCameraFusion_animate.py :** generates the sensor fusion gif  above

**/scripts/ProjectSolution_animate.py :** generates the TTc calculation gif below

**/scripts/Utils.py :**

- View class: helper class to plot results of each step
- Plot class: helper class for creating animation

![code_structure](./figures/CodeStructure.png)


![TTC](./figures/TTC_cl.gif)

## Dependencies

Tested on ubuntu 18.04 
* conda 4.8.3
* keras 2.3.1
* tensorflow 1.14.0
* recordtype 1.3
* opencv 4.3.0
* glob 
* python 3.7

tested on macos bigsur

## References

Code references were cited in the scripts.

_[1]_ https://github.com/udacity/SFND_3D_Object_Tracking

_[2]_ https://github.com/windowsub0406/KITTI_Tutorial

_[3]_ https://www.pyimagesearch.com/2018/11/12/yolo-object-detection-with-opencv/

_[4]_ https://github.com/experiencor/keras-yolo3

# 3D Object Tracking
## Sensor Fusion Engineer Nanodegree

This project aims to build prototypal components of the vehicle collision avoidance system utilizing 
LiDAR and Camera sensors and tune the parameters of these components. A Time-to-Collision (TTC) to 
the vehicle ahead is calculated separately from the LiDAR and Camera data, and an accuracy analysis 
justifies the choice of parameters.

The architecture of the system built is presented in the following picture.

![sys-arch.png](pics/sys-arch.png)

#### LiDAR-derived TTC

The YOLOv3 neural network is at the core of the "detect and classify objects" step (rectangle #2 in the picture above). 
It is famous for its ability to detect and classify objects quickly while also having a reasonable accuracy.
The code corresponding to this step resides in the function `detectObjects` from the 
[objectDetection2D.cpp](src/objectDetection2D.cpp) file.

![yolov3.png](pics/yolov3.png)

The `cropLidarPoints` function from the [lidarData.cpp](src/lidarData.cpp)  represents the "crop LiDAR points" 
step from the system architecture diagram. It removes the LiDAR points that do not belong to the lane in which 
the ego vehicle is. It is peculiar to the selected dataset and cannot be used as is in a real product.

The `clusterLidarWithROI` function from the [camFusion_Student.cpp](src/camFusion_Student.cpp) file is responsible 
for creating groups of LiDAR points whose projection into the camera image falls into the same bounding boxes 
produced by the YOLOv3. It implements the "cluster LiDAR point cloud" step from the system diagram 
(rectangle #3 in the picture above).


The matching of bounding boxes from the previous and the current frames corresponding to the vehicle ahead will 
be described in the section [Camera-derived TTC](#camera-derived-ttc) as it provides a better context to 
facilitate understanding of the algorithm.


The LiDAR-derived TTC is calculated using the function `computeTTCLidar` from the file 
[camFusion_Student.cpp](src/camFusion_Student.cpp). To avoid severe estimation errors and make such estimations 
more robust, the following approach has been adapted:
- Take the bounding boxes corresponding to the car ahead for the previous and the current frames and 
  all the LiDAR points projected into it.
- For both the previous and the current bounding boxes, disregard the closest 5 points, 
  take the next 5 points (6th to 10th), average their X-coordinates (X-axis is along the lane).
- Having a fixed frequency of LiDAR measurements, compute the speed with which the ego vehicle approaches the car ahead.
- Knowing the current distance to the car ahead and the current approach speed, one could calculate the TTC 
  by dividing the current distance by the current speed.

The following is a top-down view of the LiDAR points projected onto the car's rear bumper driving in front. 
It has one outlier that, if not filtered, will have produced a TTC estimation that is significantly 
off the actual value.

![lidar-top-down-outlier.png](pics/lidar-top-down-outlier.png)

The LiDAR-derived TTC calculation happens in the "compute TTC on object in front" step from the system diagram 
(rectangle #9 in the picture above).

#### Camera-derived TTC

The detection of image keypoints happens in the function(s) ` detKeypoints*` from the file 
[matching2D_Student.cpp](src/matching2D_Student.cpp) and corresponds to the step "detect image keypoints" 
from the system diagram (rectangle #5 in the picture above).

The calculation of keypoints descriptors happens in the function `descKeypoints` from the file 
[matching2D_Student.cpp](src/matching2D_Student.cpp) and corresponds to the step "extract keypoint descriptors" 
from the system diagram (rectangle #6 in the picture above).

The matching of keypoint descriptors from the previous and the current frames is done in the function 
`matchDescriptors` from the file [matching2D_Student.cpp](src/matching2D_Student.cpp). 
It corresponds to the step "match keypoint descriptors" from the system diagram (rectangle #7 in the picture above).

The next step is to associate the keypoints that matched previously with the bounding boxes. 
It is handled in the function `findBoundingBoxesContainingKeypoint` from the 
[camFusion_Student.cpp](src/camFusion_Student.cpp) file. This keypoints association with bounding boxes is 
the first part of the "track 3D object bounding boxes" step from the system diagram (rectangle #8 in the picture above).

The matching of bounding boxes from the previous and the current frames corresponding to the vehicle ahead is done
in the `matchBoundingBoxes` function from the file [camFusion_Student.cpp](src/camFusion_Student.cpp).
The bounding boxes between the current and the previous frames are associated based on the number of matches between
the key points lying inside corresponding bounding boxes. Which points are lying inside which boxes is determined by 
the function `findBoundingBoxesContainingKeypoint` from the [camFusion_Student.cpp](src/camFusion_Student.cpp) file.
The pair of bounding boxes having the maximum number of matches gets chosen. This bounding box matching corresponds to
the "track 3D object bounding boxes" step from the system diagram (rectangle #8 in the picture above).


To compute the Camera-derived TTC, one first needs to associate key points with the current bounding box of the vehicle 
ahead while accounting for outlier matches and removing them from consideration. 
The function `clusterKptMatchesWithROI` from the file [camFusion_Student.cpp](src/camFusion_Student.cpp) accomplishes 
this disregarding 20% of all the keypoint pairs having the highest euclidean distances between them. 
The Camera-derived TTC is computed in the function `computeTTCCamera` from the file 
[camFusion_Student.cpp](src/camFusion_Student.cpp). The following picture defines variables that explain the formula 
for calculating the camera-derived TTC that is included after it.

![cam-vars.png](pics/cam-vars.png)  
![ttc-cam-formula.png](pics/ttc-cam-formula.png)

The Camera-derived TTC calculation happens in the "compute TTC on object in front" step from the system diagram
(rectangle #9 in the picture above).

#### Accuracy Analysis

With the parameter tuning of the LiDAR-based TTC component and the choice of the most appropriate combination of 
(detector, descriptor, descriptor type, matcher, selector) for the Camera-based TTC component, 
the outlier estimates of TTC in both cases have been eliminated.

There are 252 possible (valid) combinations of (detector, descriptor, descriptor type, matcher, selector), 
and one needs to choose the most suitable combination in terms of accuracy.

To choose an optimal combination of (detector, descriptor, descriptor type, matcher, selector), a scoring system was
constructed that produces the score between 0 and 1 for each combination; the higher the score, 
the better the combination performs. It will have taken forever to find an optimal combination manually.

This score is a combination of three other scores:
1. Score assessing the number of measurements. The more measurements, the better. 
   A combination may produce NaN or Inf values for some pair of images, and such measurements have to be filtered out.
2. Score assessing how similar LiDAR-derived and Camera-derived TTC estimates are. 
   The Mean Absolute Error (MAE) metric takes part in the calculation of this score.
3. Score assessing how smooth the series of LiDAR, Camera, and Average between LiDAR and Camera estimates are. 
   This score relies on Root Mean Square Error (RMSE) and the differences between two consecutive TTC estimates. 
   The RMSE metric is used because we want to penalize more for larger TTC estimate differences. 
   RMSEs for LiDAR-derived, for Camera-derived, and for (LiDAR-TTC + Camera-TTC) / 2. 
   It would be more appropriate to use some other measure of time series smoothness, such as that defined in 
   the paper "Measuring and comparing smoothness in time series. The production smoothing hypothesis" 
   (https://doi.org/10.1016/0304-4076(94)90059-0). However, it is more laborious to understand and implement it.

All the scores are calculated using either of the following formulas:
- 1 - (curr_val - min_val_in_dataset)  / (max_val_in_dataset - min_val_in_dataset), 
  when the lower the better (MAE, RMSE).
- (curr_val - min_val_in_dataset)  / (max_val_in_dataset - min_val_in_dataset), 
  when the greater the better (# of estimates).

Some combinations are excluded from consideration at all. Those are having at least one TTC estimates that are way off. 
In the case considered here, these are TTCs < 5 seconds and TTCs > 25 seconds. 
This exclusion was done to balance the final scores of combinations. 
Otherwise, the best ten combinations were having total scores > 0.99, and the distinction between them was not 
noticeable.

All the scores are calculated in the [Statistics_and_Graphs.ipynb](Statistics_and_Graphs.ipynb) file.

The TOP-10 combinations in terms of accuracy are

| **COMBINATION** | **SCORE** | **IMG#1** | **IMG#2** | **IMG#3** | **IMG#4** | **IMG#5** | **IMG#6** | **IMG#7** | **IMG#8** | **IMG#9** | **IMG#10** | **IMG#11** | **IMG#12** | **IMG#13** | **IMG#14** | **IMG#15** | **IMG#16** | **IMG#17** | **IMG#18** |  
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|  
| SHITOMASI_SIFT_HOG_FLANN_NN | 0.971 | **L**:14.87 <br> **C**:14.28 | **L**:12.23 <br> **C**:13.04 | N/A | **L**:13.01 <br> **C**:12.87 | **L**:11.87 <br> **C**:11.96 | **L**:10.61 <br> **C**:13.58 | **L**:15.25 <br> **C**:13.29 | **L**:15.65 <br> **C**:13.99 | **L**:12.35 <br> **C**:12.04 | **L**:14.69 <br> **C**:14.08 | **L**:12.72 <br> **C**:12.00 | **L**:9.76 <br> **C**:12.01 | **L**:8.89 <br> **C**:12.08 | **L**:9.95 <br> **C**:12.47 | **L**:7.77 <br> **C**:12.46 | **L**:10.37 <br> **C**:11.99 | **L**:10.89 <br> **C**:11.34 | N/A |  
| SHITOMASI_SIFT_HOG_BF_NN | 0.910 | **L**:14.87 <br> **C**:14.07 | **L**:12.23 <br> **C**:13.74 | N/A | **L**:13.01 <br> **C**:13.02 | **L**:11.87 <br> **C**:12.41 | **L**:10.61 <br> **C**:13.74 | **L**:15.25 <br> **C**:13.36 | **L**:15.65 <br> **C**:13.99 | **L**:12.35 <br> **C**:12.11 | **L**:14.69 <br> **C**:14.08 | **L**:12.72 <br> **C**:11.67 | N/A | **L**:8.89 <br> **C**:12.08 | **L**:9.95 <br> **C**:12.47 | **L**:7.77 <br> **C**:12.46 | **L**:10.37 <br> **C**:11.72 | **L**:10.89 <br> **C**:11.86 | N/A |  
| AKAZE_FREAK_BINARY_BF_NN | 0.902 | N/A | **L**:12.23 <br> **C**:13.90 | N/A | **L**:13.01 <br> **C**:13.75 | **L**:11.87 <br> **C**:14.96 | **L**:10.61 <br> **C**:14.12 | **L**:15.25 <br> **C**:15.45 | **L**:15.65 <br> **C**:15.75 | **L**:12.35 <br> **C**:14.23 | **L**:14.69 <br> **C**:11.62 | **L**:12.72 <br> **C**:12.23 | **L**:9.76 <br> **C**:10.85 | **L**:8.89 <br> **C**:10.79 | **L**:9.95 <br> **C**:10.36 | N/A | **L**:10.37 <br> **C**:10.22 | **L**:10.89 <br> **C**:9.88 | N/A |  
| AKAZE_AKAZE_BINARY_FLANN_NN | 0.885 | **L**:14.87 <br> **C**:12.58 | **L**:12.23 <br> **C**:14.71 | N/A | **L**:13.01 <br> **C**:13.80 | **L**:11.87 <br> **C**:15.42 | **L**:10.61 <br> **C**:13.57 | **L**:15.25 <br> **C**:15.62 | **L**:15.65 <br> **C**:13.99 | **L**:12.35 <br> **C**:15.96 | **L**:14.69 <br> **C**:11.50 | **L**:12.72 <br> **C**:12.47 | **L**:9.76 <br> **C**:11.64 | **L**:8.89 <br> **C**:11.49 | **L**:9.95 <br> **C**:10.76 | N/A | **L**:10.37 <br> **C**:10.27 | **L**:10.89 <br> **C**:9.53 | N/A |  
| AKAZE_BRISK_HOG_BF_NN | 0.876 | N/A | **L**:12.23 <br> **C**:14.72 | N/A | **L**:13.01 <br> **C**:14.82 | **L**:11.87 <br> **C**:13.68 | **L**:10.61 <br> **C**:15.45 | **L**:15.25 <br> **C**:16.49 | **L**:15.65 <br> **C**:13.99 | **L**:12.35 <br> **C**:13.62 | **L**:14.69 <br> **C**:11.98 | **L**:12.72 <br> **C**:12.57 | **L**:9.76 <br> **C**:11.65 | **L**:8.89 <br> **C**:10.20 | **L**:9.95 <br> **C**:10.38 | N/A | **L**:10.37 <br> **C**:10.51 | **L**:10.89 <br> **C**:9.14 | N/A |  
| AKAZE_BRISK_BINARY_FLANN_NN | 0.862 | **L**:14.87 <br> **C**:12.87 | **L**:12.23 <br> **C**:14.85 | **L**:19.70 <br> **C**:13.34 | **L**:13.01 <br> **C**:14.80 | **L**:11.87 <br> **C**:14.07 | **L**:10.61 <br> **C**:15.30 | **L**:15.25 <br> **C**:16.67 | **L**:15.65 <br> **C**:13.69 | **L**:12.35 <br> **C**:14.02 | **L**:14.69 <br> **C**:12.02 | **L**:12.72 <br> **C**:12.90 | **L**:9.76 <br> **C**:11.70 | **L**:8.89 <br> **C**:10.16 | **L**:9.95 <br> **C**:9.84 | N/A | **L**:10.37 <br> **C**:10.54 | **L**:10.89 <br> **C**:9.36 | N/A |  
| AKAZE_BRIEF_HOG_BF_NN | 0.859 | **L**:14.87 <br> **C**:14.49 | **L**:12.23 <br> **C**:16.94 | N/A | **L**:13.01 <br> **C**:14.57 | **L**:11.87 <br> **C**:15.87 | **L**:10.61 <br> **C**:15.06 | **L**:15.25 <br> **C**:16.70 | **L**:15.65 <br> **C**:14.92 | **L**:12.35 <br> **C**:17.03 | **L**:14.69 <br> **C**:12.48 | **L**:12.72 <br> **C**:12.83 | **L**:9.76 <br> **C**:10.90 | **L**:8.89 <br> **C**:12.47 | **L**:9.95 <br> **C**:11.15 | N/A | **L**:10.37 <br> **C**:10.81 | **L**:10.89 <br> **C**:10.21 | N/A |  
| SHITOMASI_ORB_BINARY_BF_NN | 0.859 | N/A | **L**:12.23 <br> **C**:13.06 | N/A | **L**:13.01 <br> **C**:12.79 | **L**:11.87 <br> **C**:12.44 | **L**:10.61 <br> **C**:13.08 | **L**:15.25 <br> **C**:12.34 | **L**:15.65 <br> **C**:13.29 | **L**:12.35 <br> **C**:11.27 | **L**:14.69 <br> **C**:13.95 | **L**:12.72 <br> **C**:11.79 | N/A | **L**:8.89 <br> **C**:12.13 | **L**:9.95 <br> **C**:11.71 | **L**:7.77 <br> **C**:11.04 | **L**:10.37 <br> **C**:11.10 | **L**:10.89 <br> **C**:11.73 | N/A |  
| FAST_FREAK_HOG_FLANN_KNN | 0.854 | N/A | **L**:12.23 <br> **C**:13.08 | N/A | **L**:13.01 <br> **C**:12.85 | **L**:11.87 <br> **C**:13.75 | **L**:10.61 <br> **C**:12.50 | **L**:15.25 <br> **C**:12.25 | **L**:15.65 <br> **C**:11.76 | **L**:12.35 <br> **C**:13.12 | **L**:14.69 <br> **C**:14.41 | **L**:12.72 <br> **C**:12.44 | N/A | **L**:8.89 <br> **C**:12.83 | **L**:9.95 <br> **C**:12.52 | **L**:7.77 <br> **C**:10.84 | **L**:10.37 <br> **C**:12.28 | **L**:10.89 <br> **C**:11.07 | N/A |  
| AKAZE_BRISK_HOG_FLANN_NN | 0.848 | N/A | **L**:12.23 <br> **C**:15.43 | N/A | **L**:13.01 <br> **C**:14.80 | **L**:11.87 <br> **C**:13.28 | **L**:10.61 <br> **C**:14.62 | **L**:15.25 <br> **C**:17.43 | **L**:15.65 <br> **C**:14.38 | **L**:12.35 <br> **C**:13.85 | **L**:14.69 <br> **C**:11.92 | **L**:12.72 <br> **C**:12.98 | **L**:9.76 <br> **C**:11.75 | **L**:8.89 <br> **C**:10.26 | **L**:9.95 <br> **C**:10.34 | N/A | **L**:10.37 <br> **C**:10.43 | **L**:10.89 <br> **C**:9.15 | N/A |


For clarity of visualizations based on performance analysis done in the previous related project 
[https://github.com/ser94mor/2d-feature-tracking](https://github.com/ser94mor/2d-feature-tracking) 
the following combinations were selected for further analysis:
- SHITOMASI_SIFT_HOG_FLANN_NN (clear winner in terms of accuracy)
- AKAZE_BRISK_HOG_BF_NN       (AKAZE-detector is slow but BRISK-descriptor is fast)
- SHITOMASI_ORB_BINARY_BF_NN  (ORB-descriptor is much faster than SIFT)
- FAST_FREAK_HOG_FLANN_KNN    (the only FAST-detector in the TOP-10 )

LiDAR-based TTC estimates are the same for all the combinations.

![graph.png](pics/graph.png)  

From the visual graph one cannot identify significant outliers, although the LiDAR-derived TTC estimations 
look less stable. One can identify as outlier the LiDAR-based estimation at the image #15. Below are pictures 
illustrating the top-down view of the LiDAR points and LiDAR points projected to the images #14 and #15.

![outlier-14](pics/outlier-14.png)
![outlier-15](pics/outlier-15.png)

The root cause of the problem cannot be identified simply by looking onto the pictures. But 7.04 / ((7.13-7.04)/0.1) is, 
indeed, equals ~7.8.

Visually it is hard to choose the best (detector, descriptor, descriptor type, matcher, selector)  combination. 
I personally like the trends of SHITOMASI_SIFT_HOG_FLANN_NN and SHITOMASI_ORB_BINARY_BF_NN. 
The latter one look more realistic to me, 
although it has lower score than the former. Another advantage of SHITOMASI_ORB_BINARY_BF_NN is that both 
SHITOMASI detector and ORD descriptor are among the TOP-5 fastest detectors and descriptors as determined in 
the previous related project 
[https://github.com/ser94mor/2d-feature-tracking](https://github.com/ser94mor/2d-feature-tracking).

The vehicle collision avoidance system should both be precise in its TTC estimations and work in real time, 
so the combination **SHITOMASI_ORB_BINARY_BF_NN** seems like the most suitable.

#### Building and Running

The build happens inside the docker container. The docker image is included in this repository. 
One needs to execute the `do_build.sh` script to build the project. The build artifacts will be located in the 
`cmake-build` folder.  

The generated executable has all the OpenCV-dependencies statically linked to it, 
so it can be run without the docker container, on a host, by executing the `do_run.sh` script.  

The `build_and_run.sh` script combines the build and run steps. 

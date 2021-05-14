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


#### Keypoints Statistics
![keypoints_statistics.png](readme_images/keypoints_statistics.png)
To see all the details, open the image above in a new tab and zoom.

#### Matches Statistics
![matches_statistics.png](readme_images/matches_statistics.png)
To see all the details, open the image above in a new tab and zoom.

#### Timings Statistics
![timings_statistics.png](readme_images/timings_statistics.png)  
To see all the details, open the image above in a new tab and zoom.
The values near each box present the mean values.

#### Analysis

##### Number of Keypoints
Below is a list of keypoint detectors sorted in descending order 
based on the mean number of keypoints identified among 10 images. 
The analysis is based on the keypoint data presented in the section 
[Keypoints Statistics](#keypoints-statistics).   

| BRISK | AKAZE | FAST | SIFT | SHITOMASI | ORB | HARRIS |
|-------|-------|------|------|-----------|-----|--------|
| 276   | 167   | 149  | 138  | 117       | 116 | 24     |

##### Number of Matches
Below is a list of TOP-5 combinations of detector+descriptor sorted 
in descending order based on the mean number of identified matches 
among 9 pairs of 10 consecutive images. 
The analysis is based on the matches data presented in the section 
[Matches Statistics](#matches-statistics).   

| Place     | Combination(s)                                                            |  
|-----------|---------------------------------------------------------------------------|  
| 1st (278) | BRISK+SIFT, BRISK+BRIEF, BRISK+BRISK, BRISK+ORB                           |  
| 2nd (258) | BRISK+FREAK                                                               |  
| 3rd (165) | AKAZE+SIFT, AKAZE+FREAK, AKAZE+ORB, AKAZE+AKAZE, AKAZE+BRIEF, AKAZE+BRISK |  
| 4th (149) | FAST+BRIEF, FAST+SIFT, FAST+BRISK, FAST+ORB, FAST+FREAK                   |
| 5th (138) | SIFT+BRISK, SIFT+BRIEF, SIFT+SIFT                                         |

##### Timings
Below is a list of TOP-5 detectors sorted 
in descending order based on the mean time required to identify keypoints in one image. 
The analysis is based on the timings data presented in the section 
[Timings Statistics](#timings-statistics).   

| FAST    | ORB     | HARRIS  | SHITOMASI | BRISK    |  
|---------|---------|---------|-----------|----------| 
| 0.74 ms | 5.22 ms | 8.86 ms | 9.29 ms   | 27.07 ms | 

Below is a list of TOP-5 descriptors sorted 
in descending order based on the mean time required to compute a set of descriptors 
for all the keypoints in one image. 
The analysis is based on the timings data presented in the section 
[Timings Statistics](#timings-statistics).  

| BRIEF   | BRISK   | ORB     | SIFT     | AKAZE    |  
|---------|---------|---------|----------|----------|  
| 0.59 ms | 1.28 ms | 1.73 ms | 15.41 ms | 29.16 ms |  

##### TOP-3 Detector+Descriptor Combinations
Assuming that the number of keypoints, number of matches, and timings 
ratings contribute equally, the TOP-3 detector-descriptor combinations 
are chosen.  

| Place                            | Combination | Score              |
|----------------------------------|-------------|--------------------|  
| 1st place (if you value accuracy)| BRISK+BRIEF | 8 = 1 + 1 + 5 + 1  | 
| 2nd place (if you value speed)   | FAST+BRIEF  | 9 = 3 + 4 + 1 + 1  |   
| 2nd place (if you value accuracy)| BRISK+BRISK | 9 = 1 + 1 + 5 + 2  | 
| 3rd place (if you value speed)   | FAST+BRISK  | 10 = 3 + 4 + 1 + 2 | 

##### Summary
The choice between BRISK+BRIEF and FAST+BRIEF should be based 
on the hardware available and accuracy requirements. 
From visual observations it is clear that both combinations produce some 
false matches on each pair of images, that is why it is probably better 
to choose FAST+BRIEF combination for collision avoidance system, 
given the significant gap between the FAST and BRISK keypoint 
detection times.

The winner is **FAST detector + BRIEF descriptor** combination.

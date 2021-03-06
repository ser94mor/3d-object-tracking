
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "CircularBuffer.hpp"
#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;


constexpr bool kSingleRunFlag = true;
constexpr struct {
    Detector detector = detector_SHITOMASI;
    Descriptor descriptor = descriptor_ORB;
    DescriptorType descriptor_type = descriptor_type_BINARY;
    Matcher matcher = matcher_BF;
    Selector selector = selector_NN;
} kSingleRunConfig;



/* MAIN PROGRAM */
int main(int, const char*[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    for (auto e_detector : detector_array) {
        for (auto e_descriptor : descriptor_array) {
            if ((e_descriptor == descriptor_AKAZE && e_detector != detector_AKAZE) ||
                (e_descriptor == descriptor_ORB && e_detector == detector_SIFT)) {
                // AKAZE descriptor extractor works only with key-points detected with KAZE/AKAZE detectors
                // see https://docs.opencv.org/3.0-beta/modules/features2d/doc/feature_detection_and_description.html#akaze

                // ORB descriptor extractor does not work with the SIFT detetor
                // see https://answers.opencv.org/question/5542/sift-feature-descriptor-doesnt-work-with-orb-keypoinys/
                continue;
            }

            for (auto e_descriptor_type : CompatibleDescriptorTypes(e_descriptor)) {
                for (auto e_matcher : matcher_array) {
                    for (auto e_selector : selector_array) {

                        if (kSingleRunFlag)
                        {
                            e_detector = kSingleRunConfig.detector;
                            e_descriptor = kSingleRunConfig.descriptor;
                            e_descriptor_type = kSingleRunConfig.descriptor_type;
                            e_matcher = kSingleRunConfig.matcher;
                            e_selector = kSingleRunConfig.selector;
                        }

                        // misc
                        double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
                        const size_t dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
                        CircularBuffer<DataFrame, dataBufferSize> dataBuffer; // list of data frames which are held in memory at the same time
                        bool bVis = kSingleRunFlag;            // visualize results
                        std::ostringstream oss;
                        oss << ToString(e_detector) << '_'
                            << ToString(e_descriptor) << '_'
                            << ToString(e_descriptor_type) << '_'
                            << ToString(e_matcher) << '_'
                            << ToString(e_selector);

                        std::string unique_prefix = oss.str();


                        std::cout << "\n\n\n\n" << unique_prefix << std::endl;

                        std::ofstream ttc_ofs{unique_prefix + ".txt", std::ios::out};
                        ttc_ofs << "image_id ttc_lidar ttc_camera\n";

                        /* MAIN LOOP OVER ALL IMAGES */

                        for (int imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth) {
                            /* LOAD IMAGE INTO BUFFER */

                            // assemble filenames for current index
                            ostringstream imgNumber;
                            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                            // load image from file
                            cv::Mat img = cv::imread(imgFullFilename);

                            // push image into data frame buffer
                            DataFrame frame;
                            frame.cameraImg = img;
                            dataBuffer.push_back(frame);

                            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


                            /* DETECT & CLASSIFY OBJECTS */

                            float confThreshold = 0.2;
                            float nmsThreshold = 0.4;
                            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes,
                                          confThreshold, nmsThreshold,
                                          yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights,
                                          bVis);

                            cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


                            /* CROP LIDAR POINTS */

                            // load 3D Lidar points from file
                            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
                            std::vector<LidarPoint> lidarPoints;
                            loadLidarFromFile(lidarPoints, lidarFullFilename);

                            // remove Lidar points based on distance properties
                            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
                            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

                            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

                            cout << "#3 : CROP LIDAR POINTS done" << endl;


                            /* CLUSTER LIDAR POINT CLOUD */

                            // associate Lidar points with camera-based ROI
                            float shrinkFactor = 0.2; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
                            clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes,
                                                (dataBuffer.end() - 1)->lidarPoints,
                                                shrinkFactor, P_rect_00, R_rect_00, RT);

                            // Visualize 3D objects
                            bVis = kSingleRunFlag;
                            if (bVis) {
                                show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0),
                                              cv::Size(2000, 2000), imgIndex, true);
                            }
                            bVis = false;

                            cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;

                            /* DETECT IMAGE KEYPOINTS */

                            // convert current image to grayscale
                            cv::Mat imgGray;
                            cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

                            // extract 2D keypoints from current image
                            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                            string detectorType = ToString(e_detector);

                            if (detectorType == "SHITOMASI") {
                                detKeypointsShiTomasi(keypoints, imgGray, false);
                            } else if (detectorType == "HARRIS") {
                                detKeypointsHarris(keypoints, imgGray, bVis);
                            } else {
                                detKeypointsModern(keypoints, imgGray, detectorType, bVis);
                            }

                            // optional : limit number of keypoints (helpful for debugging and learning)
                            bool bLimitKpts = false;
                            if (bLimitKpts) {
                                int maxKeypoints = 50;

                                if (detectorType.compare("SHITOMASI") ==
                                    0) { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                                }
                                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                                cout << " NOTE: Keypoints have been limited!" << endl;
                            }

                            // push keypoints and descriptor for current frame to end of data buffer
                            (dataBuffer.end() - 1)->keypoints = keypoints;

                            cout << "#5 : DETECT KEYPOINTS done" << endl;


                            /* EXTRACT KEYPOINT DESCRIPTORS */

                            cv::Mat descriptors;
                            string descriptor = ToString(e_descriptor); // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
                            descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg,
                                          descriptors,
                                          descriptor);

                            // push descriptors for current frame to end of data buffer
                            (dataBuffer.end() - 1)->descriptors = descriptors;

                            cout << "#6 : EXTRACT DESCRIPTORS done" << endl;


                            if (dataBuffer.size() > 1) // wait until at least two images have been processed
                            {

                                /* MATCH KEYPOINT DESCRIPTORS */

                                vector<cv::DMatch> matches;
                                string matcherType = ToString(e_matcher);        // BF, FLANN
                                string descriptorType = ToString(e_descriptor_type); // BINARY, HOG
                                string selectorType = ToString(e_selector);       // NN, KNN

                                matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                                 (dataBuffer.end() - 2)->descriptors,
                                                 (dataBuffer.end() - 1)->descriptors,
                                                 matches, descriptorType, matcherType, selectorType);

                                // store matches in current data frame
                                (dataBuffer.end() - 1)->kptMatches = matches;

                                cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;


                                /* TRACK 3D OBJECT BOUNDING BOXES */

                                // associate bounding boxes between current and previous frame using keypoint matches
                                map<int, int> bbBestMatches;
                                matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2),
                                                   *(dataBuffer.end() - 1));

                                // store matches in current data frame
                                (dataBuffer.end() - 1)->bbMatches = bbBestMatches;

                                cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


                                /* COMPUTE TTC ON OBJECT IN FRONT */

                                // loop over all BB match pairs
                                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin();
                                     it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1) {
                                    // find bounding boxes associates with current match
                                    BoundingBox *prevBB, *currBB;
                                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin();
                                         it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2) {
                                        if (it1->second ==
                                            it2->boxID) // check wether current match partner corresponds to this BB
                                        {
                                            currBB = &(*it2);
                                        }
                                    }

                                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin();
                                         it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2) {
                                        if (it1->first ==
                                            it2->boxID) // check wether current match partner corresponds to this BB
                                        {
                                            prevBB = &(*it2);
                                        }
                                    }

                                    // compute TTC for current match
                                    if (currBB->lidarPoints.size() > 0 &&
                                        prevBB->lidarPoints.size() > 0) // only compute TTC if we have Lidar points
                                    {
                                        // compute time-to-collision based on Lidar data
                                        double ttcLidar;
                                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate,
                                                        ttcLidar);

                                        // compute time-to-collision based on camera
                                        double ttcCamera;
                                        // assign enclosed keypoint matches to bounding box
                                        clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints,
                                                                 (dataBuffer.end() - 1)->keypoints,
                                                                 (dataBuffer.end() - 1)->kptMatches);
                                        computeTTCCamera((dataBuffer.end() - 2)->keypoints,
                                                         (dataBuffer.end() - 1)->keypoints, currBB->kptMatches,
                                                         sensorFrameRate, ttcCamera);

                                        const bool is_valid = not
                                                ( std::isnan(ttcLidar)  or
                                                  std::isnan(ttcCamera) or
                                                  std::isinf(ttcLidar)  or
                                                  std::isinf(ttcCamera) );


                                        if (is_valid) {
                                            bVis = kSingleRunFlag;
                                            if (bVis) {
                                                cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                                                showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00,
                                                                    RT,
                                                                    &visImg);
                                                cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y),
                                                              cv::Point(currBB->roi.x + currBB->roi.width,
                                                                        currBB->roi.y + currBB->roi.height),
                                                              cv::Scalar(0, 255, 0), 2);

                                                char str[200];
                                                sprintf(str, "Image ID: %d, TTC Lidar : %f s, TTC Camera : %f s",
                                                        imgIndex, ttcLidar, ttcCamera);
                                                putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2,
                                                        cv::Scalar(0, 0, 255));

                                                string windowName = "Final Results : TTC";
                                                cv::namedWindow(windowName, 4);
                                                cv::imshow(windowName, visImg);
                                                cout << "Press key to continue to next frame" << endl;
                                                cv::waitKey(0);
                                            }
                                            bVis = false;

                                            ttc_ofs << imgIndex << ' ' << ttcLidar << ' ' << ttcCamera << '\n';
                                        }

                                    } // eof TTC computation
                                } // eof loop over all BB matches

                            } // end of "if" data buffer is not empty

                        } // eof loop over all images

                        if (kSingleRunFlag)
                        {
                            goto out;
                        }

                    }
                }
            }
        }
    }
out:
    return EXIT_SUCCESS;
}

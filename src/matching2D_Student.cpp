#include <numeric>
#include <fstream>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType,
                      std::string matcherType, std::string selectorType)
{
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType == "BF")
  {
    int normType = (descriptorType == "HOG") ? cv::NORM_L2 : cv::NORM_HAMMING;
    matcher = cv::BFMatcher::create(normType, crossCheck);
  }
  else if (matcherType == "FLANN")
  {
    if (descSource.type() != CV_32F)
    { // OpenCV bug workaround :
      //     convert binary descriptors to floating point due to a bug in current OpenCV implementation
      descSource.convertTo(descSource, CV_32F);
    }

    if (descRef.type() != CV_32F)
    {
      descRef.convertTo(descRef, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  // perform matching task
  if (selectorType == "NN")
  { // nearest neighbor (best match)
    auto t = static_cast<double>(cv::getTickCount());

    matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
  }
  else if (selectorType == "KNN")
  { // k nearest neighbors (k=2)
    int k = 2; // number of neighbours
    vector<vector<cv::DMatch>> knn_matches;
    auto t = static_cast<double>(cv::getTickCount());

    matcher->knnMatch(descSource, descRef, knn_matches, k); // finds the k best matches

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;


    // filter matches using descriptor distance ratio test
    double minDescDistRatio = 0.8;
    for (auto& knn_match : knn_matches)
    {
      if (knn_match[0].distance < minDescDistRatio * knn_match[1].distance)
      {
        matches.push_back(knn_match[0]);
      }
    }
    cout << "(KNN) # keypoints removed = " << knn_matches.size() - matches.size() << endl;
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType == "BRISK")
  {
    int threshold = 30;        // FAST/AGAST detection threshold score.
    int octaves = 3;           // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  }
  else if (descriptorType == "BRIEF")
  {
    int bytes = 32;               // legth of the descriptor in bytes
    bool use_orientation = false; // sample patterns using key points orientation

    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, use_orientation);
  }
  else if (descriptorType == "ORB")
  {
    int   nfeatures = 500;     // The maximum number of features to retain.
    float scaleFactor = 1.2f;  // Pyramid decimation ratio, greater than 1.
    int   nlevels = 8;         // The number of pyramid levels.
    int   edgeThreshold = 31;  // This is size of the border where the features are not detected.
    int   firstLevel = 0;      // The level of pyramid to put source image to.
    int   WTA_K = 2;           // The number of points that produce each element of the oriented BRIEF descriptor.
    auto  scoreType = cv::ORB::HARRIS_SCORE; // HARRIS_SCORE / FAST_SCORE algorithm is used to rank features.
    int   patchSize = 31;      // Size of the patch used by the oriented BRIEF descriptor.
    int   fastThreshold = 20;  // The FAST threshold.

    extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold,
                                firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
  }
  else if (descriptorType == "FREAK")
  {
    bool orientationNormalized = true; // Enable orientation normalization.
    bool scaleNormalized = true;       // Enable scale normalization.
    float patternScale = 22.0f;        // Scaling of the description pattern.
    int nOctaves = 4;                  // Number of octaves covered by the detected keypoints.
    const std::vector<int>& selectedPairs = std::vector<int>(); // (Optional) user defined selected pairs indexes.

    extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale,
                                               nOctaves, selectedPairs);
  }
  else if (descriptorType == "AKAZE")
  {
    // Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT,
    //                                   DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
    auto  descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
    int   descriptor_size = 0;        // Size of the descriptor in bits. 0 -> Full size
    int   descriptor_channels = 3;    // Number of channels in the descriptor (1, 2, 3).
    float threshold = 0.001f;         //   Detector response threshold to accept point.
    int   nOctaves = 4;               // Maximum octave evolution of the image.
    int   nOctaveLayers = 4;          // Default number of sublevels per scale level.
    auto  diffusivity = cv::KAZE::DIFF_PM_G2; // Diffusivity type. DIFF_PM_G1, DIFF_PM_G2,
    //                   DIFF_WEICKERT or DIFF_CHARBONNIER.
    extractor = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels,
                                  threshold, nOctaves, nOctaveLayers, diffusivity);
  }
  else if (descriptorType == "SIFT")
  {
    int nfeatures = 0; // The number of best features to retain.
    int nOctaveLayers = 3; // The number of layers in each octave. 3 is the value used in D. Lowe paper.
    // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions.
    double contrastThreshold = 0.04;
    double edgeThreshold = 10; // The threshold used to filter out edge-like features.
    double sigma = 1.6; // The sigma of the Gaussian applied to the input image at the octave \#0.

    extractor = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
  }
  else
  {
    throw std::invalid_argument("unknown descriptor type: " + descriptorType);
  }

  // perform feature description
  auto t = static_cast<double>(cv::getTickCount());
  extractor->compute(img, keypoints, descriptors);
  t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
  cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  // compute detector parameters based on image size
  int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0; // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

  double qualityLevel = 0.01; // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  auto t = static_cast<double>(cv::getTickCount());
  vector<cv::Point2f> corners{};
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto& corner : corners)
  {

    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f(corner.x, corner.y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

void detKeypointsHarris(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, bool bVis)
{
  // Detector parameters
  int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
  int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
  double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
  double k = 0.04; // Harris parameter

  // Detect Harris corners and normalize output
  auto t = static_cast<double>(cv::getTickCount());
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1 );
  cv::cornerHarris( img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );

  // Look for prominent corners and instantiate keypoints (NMS)
  for (int j = 0; j < dst_norm.rows; ++j)
  {
    for (int i = 0; i < dst_norm.cols; ++i)
    {
      int response = static_cast<int>(dst_norm.at<float>(j, i));
      if (response > minResponse)
      { // only store points above a threshold
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(i, j);
        newKeyPoint.size = 2 * apertureSize;
        newKeyPoint.response = response;

        // perform non-maxima suppression (NMS) in local neighbourhood around new key point
        bool bOverlap = false;
        for (auto& keypoint : keypoints)
        {
          double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, keypoint);
          if (kptOverlap > maxOverlap)
          {
            bOverlap = true;
            if (newKeyPoint.response > keypoint.response)
            {                          // if overlap is >t AND response is higher for new key point
              keypoint = newKeyPoint;  // replace old key point with new one
              break;                   // and quit loop over key points
            }
          }
        }

        if (!bOverlap)
        {                                    // only add new key point if no overlap has been found in previous NMS
          keypoints.push_back(newKeyPoint);
        }
      }
    } // end of loop over cols
  } // end of loop over rows
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // visualize results, if requested
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = "Harris Corner Detector with a Non-maximum Suppression  Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

void detKeypointsModern(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, std::string& detectorType, bool bVis)
{
  cv::Ptr<cv::FeatureDetector> detector;
  if (detectorType == "FAST")
  {
    int threshold = 30;    // difference between intensity of the central pixel and pixels of a circle around this pixel
    bool bNMS = true;      // perform non-maxima suppression on keypoints
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
    detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
  }
  else if (detectorType == "BRISK")
  {
    int threshold = 30;        //   AGAST detection threshold score
    int octaves = 3;           // detection octaves
    float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint
    detector = cv::BRISK::create(threshold, octaves, patternScale);
  }
  else if (detectorType == "ORB")
  {
    int   nfeatures = 500;     // The maximum number of features to retain.
    float scaleFactor = 1.2f;  // Pyramid decimation ratio, greater than 1.
    int   nlevels = 8;         // The number of pyramid levels.
    int   edgeThreshold = 31;  // This is size of the border where the features are not detected.
    int   firstLevel = 0;      // The level of pyramid to put source image to.
    int   WTA_K = 2;           // The number of points that produce each element of the oriented BRIEF descriptor.
    auto  scoreType = cv::ORB::HARRIS_SCORE; // HARRIS_SCORE / FAST_SCORE algorithm is used to rank features.
    int   patchSize = 31;      // Size of the patch used by the oriented BRIEF descriptor.
    int   fastThreshold = 20;  // The FAST threshold.
    detector = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold,
                               firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
  }
  else if (detectorType == "AKAZE")
  {
    // Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT,
    //                                   DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
    auto  descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
    int   descriptor_size = 0;        // Size of the descriptor in bits. 0 -> Full size
    int   descriptor_channels = 3;    // Number of channels in the descriptor (1, 2, 3).
    float threshold = 0.001f;         //   Detector response threshold to accept point.
    int   nOctaves = 4;               // Maximum octave evolution of the image.
    int   nOctaveLayers = 4;          // Default number of sublevels per scale level.
    auto  diffusivity = cv::KAZE::DIFF_PM_G2; // Diffusivity type. DIFF_PM_G1, DIFF_PM_G2,
    //                   DIFF_WEICKERT or DIFF_CHARBONNIER.
    detector = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels,
                                 threshold, nOctaves, nOctaveLayers, diffusivity);
  }
  else if (detectorType == "SIFT")
  {
    int nfeatures = 0; // The number of best features to retain.
    int nOctaveLayers = 3; // The number of layers in each octave. 3 is the value used in D. Lowe paper.
    // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions.
    double contrastThreshold = 0.04;
    double edgeThreshold = 10; // The threshold used to filter out edge-like features.
    double sigma = 1.6; // The sigma of the Gaussian applied to the input image at the octave \#0.

    detector = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
  }
  else
  {
    throw std::invalid_argument("unknown detector type: " + detectorType);
  }

  auto t = static_cast<double>(cv::getTickCount());
  detector->detect(img, keypoints);
  t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
  cout << detectorType << " with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = detectorType + " Keypoint Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

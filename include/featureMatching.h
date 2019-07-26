#pragma once

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class FeatureMatching {
  
public:
  FeatureMatching();
  ~FeatureMatching();
public:
  bool featureDetection(cv::Mat img1,
		   cv::Mat img2,
		   std::vector<cv::KeyPoint> &keyPts,
		   cv::Mat &keyDescrip);
  bool featureMatching(cv::Mat img1,
		  cv::Mat img2,
		  std::vector<cv::KeyPoint> &keyPts1,
		  std::vector<cv::KeyPoint> &keyPts2,
		  std::vector<uint16_t> &index1,
		  std::vector<uint16_t> &index2);
  bool getFeaturePts(std::vector<cv::KeyPoint> keyPts,
		     std::vector<uint16_t> index,
		     std::vector<cv::Point2f> &pts);
  bool getFeature3DPts(pcl::PointCloud<pcl::PointXYZRGBA> &cloudin,
		      std::vector<uint16_t> index,
		      std::vector<cv::Point3f> &pts);
  
};
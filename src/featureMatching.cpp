#include "featureMatching.h"

FeatureMatching::FeatureMatching()
{

}
FeatureMatching::~FeatureMatching()
{

}


bool FeatureMatching::featureDetection(cv::Mat img1, cv::Mat img2, std::vector< cv::KeyPoint >& keyPts, cv::Mat& keyDescrip)
{

  return true;
}

bool FeatureMatching::featureMatching(cv::Mat img1, cv::Mat img2, std::vector< cv::KeyPoint >& keyPts1, std::vector< cv::KeyPoint >& keyPts2, std::vector< uint16_t >& index1, std::vector< uint16_t >& index2)
{
  //SIFT
  cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(0, 3, 0.01,100);
  
  // detect keypoints & descriptor
  cv::Mat descrip1,descrip2;
  sift->detectAndCompute(img1,cv::Mat(),keyPts1, descrip1);
  sift->detectAndCompute(img2,cv::Mat(), keyPts2, descrip2);
  
  // match
  std::vector<std::vector<cv::DMatch>> initMatch;
  cv::Ptr<cv::BFMatcher> bfmatch = cv::BFMatcher::create();
  bfmatch->knnMatch(descrip1, descrip2, initMatch, 3);
  
  // ransac good keypoints
  float ratio = 0.6;
  for (int i=0; i<initMatch.size();++i)  {
    if (initMatch[i][0].distance < 0.6 * initMatch[i][1].distance) {
      index1.push_back(initMatch[i][0].queryIdx);
      index2.push_back(initMatch[i][0].trainIdx);
    }
  }
  
  // F outliers
  return true;
}

bool FeatureMatching::getFeaturePts(std::vector< cv::KeyPoint > keyPts, std::vector< uint16_t > index, std::vector< cv::Point2f > &pts)
{
    for (int i=0; i<index.size();++i)  {
      pts.push_back(keyPts[index[i]].pt);
    }
 return true;
}

bool FeatureMatching::getFeature3DPts(pcl::PointCloud<pcl::PointXYZRGBA> &cloudin, std::vector< uint16_t > index, std::vector< cv::Point3f > &pts)
{
    for (int i=0; i<index.size();++i)  {
      double x = cloudin.points[i].x;
      double y = cloudin.points[i].y;
      double z = cloudin.points[i].z;
      
      pts.push_back(cv::Point3f(x,y,z));
    }
    
    return true;
}


#include <iostream>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl/registration/registration.h>

#include <eigen3/Eigen/Eigen>

#include "imagepreprocess.h"
#include "featureMatching.h"


  
  

int main() {
  
  cv::Mat lastImg;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  FeatureMatching fm;
  
  for (int idx = 1; idx < 10; idx ++) {
    //defint path
    char rgb[1024];
    char depth[1024];
    
    sprintf(rgb, "data/in/rgb_png/%d.png",idx);
    sprintf(depth, "data/in/depth_png/%d.png",idx);
    
    // 2D to 3D
    // define K matrix
    const double fx = 518.0;
    const double fy = 519.0;
    const double cx = 325.5;
    const double cy = 253.5;
    const double scale = 1000;
    cv::Mat cameraMatrix = cv::Mat::zeros(3,3,CV_64FC1);
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;
    
    // define mat
    ImagePreProcess imgprep;
    cv::Mat rgb_img, depth_img;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // load and convert
    imgprep.loadData(rgb, depth, rgb_img, depth_img);
    imgprep.imageToPointCloud(rgb_img, depth_img, cameraMatrix, cloud_in);
    
    
    //judge
    if (idx < 2) {
      lastImg = rgb_img.clone();
      *lastCloud = *cloud_in;
    } else {
      
      std::vector<cv::KeyPoint> keypts1, keypts2;
      std::vector<uint16_t> index1, index2;
      fm.featureMatching(lastImg, rgb_img, keypts1, keypts2, index1, index2);
      
      //get 2D pts 3D pts
      std::vector<cv::Point2f> img2D;
      std::vector<cv::Point3f> obj3D;
      fm.getFeaturePts(keypts2, index2, img2D);
      fm.getFeature3DPts(*lastCloud, index1, obj3D);
      
      cv::Mat r, t, R;
      cv::Mat cof = cv::Mat::zeros(1, 8, CV_64FC1);
      cv::Mat inlier;
      cv::solvePnPRansac(obj3D, img2D, cameraMatrix, cof, r, t, false, 100, 1.0, 0.99, inlier);
      cv::Rodrigues(r,R);
      
      Eigen::
      
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clout_out ( new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::transformPointCloud(&cloud_in, *clout_out, T);
      
      // update
      lastImg = rgb_img.clone();
      lastCloud->clear();
      *lastCloud = *cloud_in;
      
    }
    
    // save
    char out[1024];
    sprintf(out,"data/out/%d_pointcloud.pcd",idx);
    std::cout<<"  save the pointcloud to "<<out<<std::endl;
    pcl::io::savePCDFile(out,*cloud_in);
    cloud_in->clear();
  }

  return 1;
}
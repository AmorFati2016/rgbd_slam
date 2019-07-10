#include <iostream>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/pcd_io.h>



#include "imagepreprocess.h"

int main() {
  
  
  
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
  
  //defint path
  char rgb[1024] = "data/in/rgb.png";
  char depth[1024] = "data/in/depth.png";
  ImagePreProcess imgprep;
  cv::Mat rgb_img, depth_img;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);
  imgprep.loadData(rgb, depth, rgb_img, depth_img);
  imgprep.imageToPointCloud(rgb_img, depth_img, cameraMatrix, cloud_in);
  
  std::cout<<" input cloud point size is : "<<depth_img.cols * depth_img.rows<<"   "<<cloud_in->points.size()<<std::endl;

  pcl::io::savePCDFile("data/out/pts.pcd",*cloud_in);

  return 1;
}
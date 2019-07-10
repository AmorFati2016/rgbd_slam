/*
 * Copyright (c) 2019, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "imagepreprocess.h"

ImagePreProcess::ImagePreProcess()
{

}
ImagePreProcess::~ImagePreProcess()
{

}
bool ImagePreProcess::loadData(char path_rgb[], char path_depth[], cv::Mat& rgbMat, cv::Mat& depthMat)
{

  // load rgb & depth
  rgbMat = cv::imread(path_rgb,1);
  depthMat = cv::imread(path_depth,-1);
  
  // check
  if (rgbMat.empty() | depthMat.empty()) {
   std::cout<<"  input rgbmat or depth mat is empty!"<<std::endl; 
   return false;
  }
  
  if (rgbMat.size()!=depthMat.size()) {
    
   std::cout<<"  input rgbmat.size() is not equal to depth mat size()"<<std::endl;
   return false;
  }
  
  return true;
  
}

bool ImagePreProcess::imageToPointCloud(cv::Mat rgbMat, cv::Mat depthMat, cv::Mat cameraK, pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud_out)
{
  
    // 2D to 3D
  // define K matrix
  const double fx = cameraK.at<double>(0,0);
  const double fy = cameraK.at<double>(1,1);
  const double cx = cameraK.at<double>(0,2);
  const double cy = cameraK.at<double>(1,2);
  const double scale = 1000;
  
//compute 3D point and save
  pcl::PointCloud<Pts3DT>::Ptr cloud_in (new pcl::PointCloud<Pts3DT>);
  
  for (int v=0; v<depthMat.rows; ++v) {
    ushort *pDepth = depthMat.ptr<ushort>(v);
    for (int u = 0; u<depthMat.cols; ++u) {

      double d = pDepth[u];
      if (d == 0) continue;

      Pts3DT pts3D;
      pts3D.z = d / scale;
      pts3D.x = (u - cx) * pts3D.z / fx;
      pts3D.y = (v - cy) * pts3D.z / fy;
      

      pts3D.b = rgbMat.ptr<uchar>(v)[3*u];
      pts3D.g = rgbMat.ptr<uchar>(v)[3*u + 1];
      pts3D.r = rgbMat.ptr<uchar>(v)[3*u + 2];

      // save the 3D point_types
      cloud_in->points.push_back(pts3D);
    }
  }
  
  //  resize the point cloud size
  cloud_in->height = 1;
  cloud_in->width = cloud_in->points.size();
  cloud_in->is_dense = false;
  *cloud_out = *cloud_in;
  
  return true;
}

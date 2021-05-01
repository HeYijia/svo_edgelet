// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <svo/math_lib.h>
#include <svo/camera_model.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <iostream>

#include <svo/slamviewer.h>
#include<thread>

namespace svo {

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

class BenchmarkNode
{
  svo::AbstractCamera* cam_;
  svo::AbstractCamera* cam_r_;
  svo::PinholeCamera* cam_pinhole_;
  svo::FrameHandlerMono* vo_;

  SLAM_VIEWER::Viewer* viewer_;
  std::thread * viewer_thread_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{

  cam_ = new svo::PinholeCamera(752, 480, 435.2046959714599, 435.2046959714599, 367.4517211914062,252.2008514404297);
  cam_r_ = new svo::PinholeCamera(752, 480,435.2046959714599, 435.2046959714599, 367.4517211914062, 252.2008514404297);

  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();

  viewer_ = new SLAM_VIEWER::Viewer(vo_);
  viewer_thread_ = new std::thread(&SLAM_VIEWER::Viewer::run,viewer_);
  viewer_thread_->detach();

}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
  delete cam_r_;
  // delete cam_pinhole_;

  delete viewer_;
  delete viewer_thread_;
}

//#define TXTREAD
void BenchmarkNode::runFromFolder()
{

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimeStamp;
  LoadImages("/media/hyj/dataset/datasets/MH_01_easy/mav0/cam0/data", "/media/hyj/dataset/datasets/MH_01_easy/mav0/cam1/data", "MH01.txt", vstrImageLeft, vstrImageRight, vTimeStamp);

  if(vstrImageLeft.empty() || vstrImageRight.empty())
  {
      //cerr << "ERROR: No images in provided path." << endl;
      return ;
  }

  if(vstrImageLeft.size()!=vstrImageRight.size())
  {
      //cerr << "ERROR: Different number of left and right images." << endl;
      return ;
  }

  // Read rectification parameters
  cv::FileStorage fsSettings("EuRoC.yaml", cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      //cerr << "ERROR: Wrong path to settings" << endl;
      return ;
  }

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;

  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;

  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
          rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
  {
      //cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
      return ;
  }

  cv::Mat M1l,M2l,M1r,M2r;
  cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);

  const int nImages = vstrImageLeft.size();

  cv::Mat imLeft, imRight;
  for(int ni=100; ni<nImages; ni++)
  {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);

    assert(!imLeft.empty());

    cv::Mat imLeft_rect;
    cv::remap(imLeft,imLeft_rect,M1l,M2l,cv::INTER_LINEAR);

    vo_->addImage(imLeft_rect, 0.01*ni);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
        std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \n";

        // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        // std::cout<<"Frame pose: "<< vo_->lastFrame()->T_f_w_ <<std::endl;

    }
  }

}

} // namespace svo

int main(int argc, char** argv)
{

    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();

  printf("BenchmarkNode finished.\n");
  return 0;
}


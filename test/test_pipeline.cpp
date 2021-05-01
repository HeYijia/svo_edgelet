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

void LoadImages(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    std::ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    std::string s0;
    std::getline(f,s0);
    std::getline(f,s0);
    std::getline(f,s0);

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
            //ss >> sRGB;
            std::string sRGB;
            ss >> sRGB;
            //sRGB = s + ".jpg";
            //std::cout<<sRGB.c_str()<<std::endl;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

class BenchmarkNode
{
  svo::AbstractCamera* cam_;
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

  //tum rgbd dataset fr2
  cam_ = new svo::PinholeCamera(640, 480, 520.9,521.0,325.1,249.7,0.2312,-0.7849,-0.0033,-0.0001,0.9172);
  //tum rgbd dataset fr1
  //cam_ = new svo::PinholeCamera(640, 480, 517.3,516.5,318.6,	255.3,	0.2624,	-0.9531,-0.0054,0.0026,	1.1633);

  // my camera
  //cam_ = new svo::PinholeCamera(640, 480, 446.867338 ,446.958766, 298.082779, 234.334299, -0.324849, 0.1205156, -0.000186, -0.000821);
  // ICL dataset
  //cam_ = new svo::PinholeCamera(640, 480, 481.20, 480.00, 319.50, 239.50);

  // kitti
  //cam_ = new svo::PinholeCamera(1226, 370, 707.0912, 707.0912, 601.8873, 183.1104);
  // tum mono dataset
  //cam_ = new svo::PinholeCamera(1226, 370, 707.0912, 707.0912, 601.8873, 183.1104);
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
  // delete cam_pinhole_;

  delete viewer_;
  delete viewer_thread_;
}

#define TXTREAD
void BenchmarkNode::runFromFolder()
{
#ifdef TXTREAD  // read image filename with txt, TUM rgbd datasets
  std::vector<std::string> vstrImageFilenames;
  std::vector<double> vTimestamps;
  std::string filepath = std::string("/media/hyj/dataset/datasets/freiburg2_desk");
  std::string strFile = filepath + "/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();
  cv::Mat img;

  for(int ni=0; ni<nImages; ni++)
  {
          std::string img_path = filepath+"/"+vstrImageFilenames[ni];
          img = cv::imread(img_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
          assert(!img.empty());

          // process frame
          //cv::Mat unimg;
          //cam_pinhole_->undistortImage(img,unimg);
          //vo_->addImage(unimg, vTimestamps[ni]);
          vo_->addImage(img, vTimestamps[ni]);

          if(vo_->lastFrame() != NULL)
          {

            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                        << "#Features: " << vo_->lastNumObservations() << " \n";

          }
  }
#else  // read image filename with id, ICL-NUIM rgbd datasets
  for(int img_id =10; img_id < 2700; ++img_id)
  {
    // load image
    std::stringstream ss;

    ss << "/media/hyj/dataset/datasets/living_room_traj3_frei_png/rgb/" << img_id << ".png";  //26_1, 28_6, 11_3

    if(img_id == 1)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE));

    assert(!img.empty());

    // process frame
    //cv::Mat unimg;
    //cam_pinhole_->undistortImage(img,unimg);
    //vo_->addImage(unimg, 0.01*img_id);
    vo_->addImage(img, 0.01*img_id);


    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \n";
                  //<< "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";
    }
  }
#endif
}

} // namespace svo

int main(int argc, char** argv)
{
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();

  return 0;
}


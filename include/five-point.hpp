/*  Copyright (c) 2013, Bo Li, prclibo@gmail.com
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * Neither the name of the copyright holder nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef FIVE_POINT_HPP
#define FIVE_POINT_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//using namespace cv;

namespace svo {

cv::Mat findEssentialMat( cv::InputArray points1, cv::InputArray points2, double focal = 1.0, cv::Point2d pp = cv::Point2d(0, 0),
                                        int method = CV_RANSAC,
                                        double prob = 0.999, double threshold = 1, cv::OutputArray mask = cv::noArray() );

void decomposeEssentialMat( const cv::Mat & E,cv::Mat & R1, cv::Mat & R2, cv::Mat & t );

int recoverPose( const cv::Mat & E, cv::InputArray points1, cv::InputArray points2, cv::Mat & R, cv::Mat & t,
                                        double focal = 1.0, cv::Point2d pp = cv::Point2d(0, 0),
                                        cv::InputOutputArray mask = cv::noArray());

}


#endif

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <queue>
#include <vector>
#include<stdio.h>
#include <string>
#include<iostream>
#include <numeric>

#include <ctime>

using namespace cv;
using namespace std;

int main()
{
    cv::Mat frame, gray;
    frame = cv::imread("/home/hyj/bagfiles/1/frame0000.jpg");
    cv::cvtColor(frame,gray, CV_BGR2GRAY);
    std::vector< cv::KeyPoint> kpts;
    cv::FastFeatureDetector fast(60);
    //fast.create("Grid");
    fast.detect(gray,kpts);
    drawKeypoints(gray,kpts,gray,cv::Scalar(255));

    // grid
    //cv::GridAdaptedFeatureDetector grid_fast(fast,200,20,20);
    //grid_fast.detect(gray,kpts);
   // drawKeypoints(frame,kpts,gray,cv::Scalar(255));
    imshow("frame",gray);

    cv::waitKey(0);
    return 0;

}

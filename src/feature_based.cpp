#include "feature_based.h"
#include <svo/feature_detection.h>
#include <svo/sparse_img_align.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/config.h>

# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"

namespace svo {

bool Feature_based_track::track(const FramePtr old_frame, const FramePtr new_frame)
{
    FramePtr new_frame_temple;
    new_frame_temple.reset(new Frame( new_frame->cam_, new_frame->img().clone(), new_frame->timestamp_));
    Features new_features;
    fts_detector_->detect(new_frame_temple.get(), new_frame_temple->img_pyr_,
                              Config::triangMinCornerScore(), new_features);

     std::vector< cv::KeyPoint > keypoints_1, keypoints_2;
     cv::Mat descriptors_1,descriptors_2;
     for(Features::iterator it=old_frame->fts_.begin(); it!=old_frame->fts_.end(); ++it)
     {
         cv::KeyPoint  Kpt;
         Kpt.octave = (*it)->level;
         Kpt.size = 7;
         Kpt.response = 15;
         Kpt.pt = cv::Point2f((*it)->px[0], (*it)->px[1]);
         keypoints_1.push_back(Kpt);
     }
     for(Features::iterator it=new_features.begin(); it!=new_features.end(); ++it)
     {
         cv::KeyPoint  Kpt;
         Kpt.octave = (*it)->level;
         Kpt.size = 7;
         Kpt.response = 15;
         Kpt.pt = cv::Point2f((*it)->px[0], (*it)->px[1]);
         keypoints_2.push_back(Kpt);
     }

     //std::vector< cv::KeyPoint > _keypoints1, _keypoints2;
/*
     cv::Ptr<cv::FeatureDetector> detector;
     detector = cv::FeatureDetector::create("FAST");
     detector->detect(old_frame->img_pyr_[0], keypoints_1);
     detector->detect(new_frame_temple->img_pyr_[0], keypoints_2);
*/
     cv::Ptr<cv::DescriptorExtractor> extractor;
     extractor = cv::DescriptorExtractor::create("ORB");

     extractor->compute(old_frame->img_pyr_[0], keypoints_1, descriptors_1);
     extractor->compute(new_frame_temple->img_pyr_[0], keypoints_2, descriptors_2);

     cv::BFMatcher matcher( cv::NORM_HAMMING, true );
     std::vector< cv::DMatch > matches;
     matcher.match( descriptors_1, descriptors_2, matches );

     vector<cv::DMatch> good_matches;
     double max_dist =0;double min_dist = 100.00;
     for(int i=0; i< matches.size(); i++)
     {
         double dist = matches[i].distance;
         if( dist < min_dist ) min_dist = dist;
         if( dist > max_dist ) max_dist = dist;
     }

     for(int i =0; i< matches.size(); i++)
     {
         if(matches[i].distance <= max(3* min_dist, 0.02))
         {
             good_matches.push_back(matches[i]);
         }
     }
    //-- Draw matches
    cv::Mat img_matches;
    cv::drawMatches( old_frame->img_pyr_[0], keypoints_1, new_frame_temple->img_pyr_[0], keypoints_2, good_matches, img_matches );
    imshow("Matches", img_matches );
    cv::waitKey(0);

/*
     for(Features::iterator it=new_features.begin(); it!=new_features.end(); ++it)
     {
       if((*it)->type == Feature::EDGELET)
         cv::line(new_frame_temple->img_pyr_[0],
                  cv::Point2f((*it)->px[0]+3*(*it)->grad[1], (*it)->px[1]-3*(*it)->grad[0]),
                  cv::Point2f((*it)->px[0]-3*(*it)->grad[1], (*it)->px[1]+3*(*it)->grad[0]),
                  cv::Scalar(0,0,255), 2);
       else
         cv::rectangle(new_frame_temple->img_pyr_[0],
                       cv::Point2f((*it)->px[0]-2, (*it)->px[1]-2),
                       cv::Point2f((*it)->px[0]+2, (*it)->px[1]+2),
                       cv::Scalar(0,255,0), CV_FILLED);
     }
    cout<<"new_frame featrue size : " << new_frame->fts_.size()<<std::endl;
    cv::imshow("new_frame_temp",new_frame_temple->img_pyr_[0]);
    cv::imshow("new_frame show",new_frame->img_pyr_[0]);
    cv::waitKey(0);
*/
    return true;
}

}

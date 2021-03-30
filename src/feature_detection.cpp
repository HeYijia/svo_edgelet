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

#include <svo/feature_detection.h>
#include <svo/feature.h>
#include <svo/fast.h>

namespace svo {
namespace feature_detection {


void saveMatToCsv(cv::Mat data, std::string filename)
{
    std::ofstream outputFile(filename.c_str());
    outputFile << cv::format(data,cv::Formatter::FMT_CSV)<<std::endl;
    outputFile.close();
}

AbstractDetector::AbstractDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        cell_size_(cell_size),
        n_pyr_levels_(n_pyr_levels),
        grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)),
        grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)),
        grid_occupancy_(grid_n_cols_*grid_n_rows_, false)
{}

void AbstractDetector::resetGrid()
{
  std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}

void AbstractDetector::setExistingFeatures(const Features& fts)
{
  std::for_each(fts.begin(), fts.end(), [&](Feature* i){
    grid_occupancy_.at(
        static_cast<int>(i->px[1]/cell_size_)*grid_n_cols_
        + static_cast<int>(i->px[0]/cell_size_)) = true;
  });
}

void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
  grid_occupancy_.at(
      static_cast<int>(px[1]/cell_size_)*grid_n_cols_
    + static_cast<int>(px[0]/cell_size_)) = true;
}

FastDetector::FastDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

void FastDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
  Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
  //Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,0,0,0.0f));
  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);
    vector<fast::fast_xy> fast_corners;
#if __SSE2__
      fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);   // 20
#elif HAVE_FAST_NEON
      fast::fast_corner_detect_9_neon(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);
#else
      fast::fast_corner_detect_10(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);
#endif
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners,8, scores);// 20
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
      if(grid_occupancy_[k])
        continue;
      const float score = fast::shiTomasiScore(img_pyr[L], xy.x, xy.y);
      if(score > corners.at(k).score)
        corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
    }
  }

  int debug=0;
  // Create feature for every corner that has high enough corner score
  std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
    if(c.score > detection_threshold)
      fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
  });

  resetGrid();
}


EdgeDetector::EdgeDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

class Pt
{
public:
    float grad;
    Vector2d xy;
    Pt()
    {
        xy[0]=0;
        xy[1]=0;
        grad = 0.0;
    }
    Pt(float grad_, Vector2d xy_): grad(grad_),xy(xy_){}

    bool operator < (const Pt &m)const{ return grad > m.grad; }
};

class Edgelete
{
public:
    float grad;
    Vector2d xy;
    Vector2d dir;
    Edgelete()
    {
        xy[0]=0;
        xy[1]=0;
        grad = 0.0;
    }
    Edgelete(float grad_, Vector2d xy_, Vector2d dir_): grad(grad_),xy(xy_), dir(dir_){}

    bool operator < (const Edgelete &m)const{ return grad > m.grad; }
};
/*
#define EDGE
void EdgeDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
#ifdef EDGE
  setExistingFeatures(fts);  // corners

  cv::Mat img = img_pyr[0].clone();
  cv::Mat gradx = cv::Mat::zeros( img_pyr[0].rows, img_pyr[0].cols, CV_32F);
  cv::Mat grady = cv::Mat::zeros(img_pyr[0].rows, img_pyr[0].cols, CV_32F);
  cv::Mat mag =  cv::Mat::zeros(img_pyr[0].rows, img_pyr[0].cols, CV_32F);
  //cv::Mat ori = cv::Mat::zeros(img_pyr[0].rows, img_pyr[0].cols, CV_32F);

  cv::GaussianBlur( img_pyr[0], img, cv::Size( 3, 3 ), 0, 0 );

  cv::Scharr(img, gradx, CV_32F, 1, 0, 1/32.0);
  cv::Scharr(img, grady, CV_32F, 0, 1, 1/32.0);

  cv::magnitude(gradx,grady,mag);

  //cv::phase(gradx,grady,ori,true);
  //cv::Mat canny;
  //cv::Canny(img,canny,20,50);

  vector< Pt > maxgrad_list1;
  vector< Pt > maxgrad_list2;
  vector< Pt > maxgrad_list3;
  vector< Pt > maxgrad_list4;

  vector < Edgelete > edge_list1;
  vector < Edgelete > edge_list2;
  vector < Edgelete > edge_list3;
  vector < Edgelete > edge_list4;

  int region_featrue_cnt[4] = {0,0,0,0};
  for(size_t k = 0; k<grid_occupancy_.size(); k++)
  {


    int n = std::floor(k/grid_n_cols_);
    // discard the cell at the boundary
    if( n == 0 || n == (grid_n_rows_-1)  || (k%grid_n_cols_) == 0 || ((k+1)%grid_n_cols_ == 0 ))
      continue;
    // calculate the start pixel coordinate  of  k'th cell
    int x_start = (k - n*grid_n_cols_)* cell_size_;
    int y_start = n*cell_size_;
    // if  no fast corner at the cell, we will detect the edge feature


    if(grid_occupancy_[k])
    {
      if(x_start <= img.cols/2 && y_start <= img.rows/2)
      {
            region_featrue_cnt[0]++;
      }else if(x_start > img.cols/2 && y_start < img.rows/2)
      {
            region_featrue_cnt[1]++;
      }else if(x_start < img.cols/2 && y_start > img.rows/2)
      {
            region_featrue_cnt[2]++;
      }else if(x_start >= img.cols/2 && y_start >= img.rows/2)
      {
            region_featrue_cnt[3]++;
      }
      continue;
    }



    float max_grad = 0;
    int maxgrad_x = 0;
    int maxgrad_y = 0;
    float gx = 0;
    float gy = 0;

     cv::Mat canny;
     cv::Rect roi(x_start,y_start,cell_size_,cell_size_);
     cv::Canny(img(roi) , canny , 30,50);

    float max_grad_2 = 0;
    int maxgrad_x2 = 0;
    int maxgrad_y2 = 0;

    for (int i=0;i<cell_size_;i++ )
      for(int j=0;j<cell_size_;j++)
    {

        float temp = mag.ptr<float>(y_start + i)[x_start+j] ;
        if( temp> max_grad_2 )
        {
          maxgrad_x2 = x_start+j;
          maxgrad_y2 = y_start + i;
          max_grad_2 = temp;
        }

           // if(canny.ptr<uchar>(y_start+i)[x_start+j] == 0) continue;
            if(canny.ptr<uchar>(i)[j] == 0) continue;
            if( temp> max_grad )
            {
              maxgrad_x = x_start+j;
              maxgrad_y = y_start + i;
              max_grad = temp;
              gx = gradx.ptr<float>(maxgrad_y)[maxgrad_x] ;
              gy = grady.ptr<float>(maxgrad_y)[maxgrad_x] ;
            }
            //sum_grad += temp;
    }
    //sum_grad /= cell_size_*cell_size_;
    //if(max_grad > sum_grad + 3 && max_grad > 5)

    int edge_threshold = 4;
    //if( max_grad > edge_threshold)
    {

          Vector2d g = Vector2d(gx,gy);
          g.normalize();

       // add neigbour grad pixe to  judge  max_grad is  edge or grad point
//          int x1 = maxgrad_x+(int)3*g[1];
//          int y1 = maxgrad_y - (int)3*g[0];
//          int x2 = maxgrad_x- (int)3*g[1] ;
//          int y2 = maxgrad_y+(int)3*g[0];
//         if( max_grad > edge_threshold && (mag.ptr<float>(y1)[x1] > edge_threshold && mag.ptr<float>(y2)[x2] > edge_threshold)
//              )
       if( max_grad > edge_threshold)
         {
            Edgelete e(max_grad,Vector2d(maxgrad_x, maxgrad_y),g);

            if(x_start <= img.cols/2 && y_start <= img.rows/2)
            {
                   edge_list1.push_back(e);
            }else if(x_start > img.cols/2 && y_start < img.rows/2)
            {
                   edge_list2.push_back(e);
            }else if(x_start < img.cols/2 && y_start > img.rows/2)
            {
                   edge_list3.push_back(e);
            }else if(x_start >= img.cols/2 && y_start >= img.rows/2)
            {
                   edge_list4.push_back(e);
            }

            //fts.push_back(new Feature(frame, Vector2d(maxgrad_x, maxgrad_y),g, 0));   // edge
          }
          else if( max_grad_2 >0.0)
          {
              Pt p(max_grad_2,Vector2d(maxgrad_x2,maxgrad_y2));
             // maxgrad_list.push_back(p);
             if(x_start <= img.cols/2 && y_start <= img.rows/2)
             {
                    maxgrad_list1.push_back(p);
             }else if(x_start > img.cols/2 && y_start < img.rows/2)
             {
                    maxgrad_list2.push_back(p);
             }else if(x_start < img.cols/2 && y_start > img.rows/2)
             {
                    maxgrad_list3.push_back(p);
             }else if(x_start >= img.cols/2 && y_start >= img.rows/2)
             {
                    maxgrad_list4.push_back(p);
             }
        //     fts.push_back(new Feature(frame, Vector2d(maxgrad_x2,maxgrad_y2), 0));
          }

    }

  }


  int num_feature = 40;
  int n = num_feature - region_featrue_cnt[0];
  if( n > 0)
  {

    sort(edge_list1.begin(), edge_list1.end());
    int a = (n>edge_list1.size())? edge_list1.size():n;
    for( int i = 0; i< a; i++)
      fts.push_back(new Feature(frame, edge_list1[i].xy, edge_list1[i].dir, 0));   // edge

    //  if edgelete is not enought, we select the max grad point in each grid
    n = n - a ;
    if(n>0)
    {

      sort(maxgrad_list1.begin(), maxgrad_list1.end());
      for( int i = 0; i< n; i++)
      {
            fts.push_back(new Feature(frame,maxgrad_list1[i].xy, 0));
      }
    }

  }

   n = num_feature - region_featrue_cnt[1];
  if( n > 0)
  {

    sort(edge_list2.begin(), edge_list2.end());
    int a = (n>edge_list2.size())? edge_list2.size():n;
    for( int i = 0; i< a; i++)
      fts.push_back(new Feature(frame, edge_list2[i].xy, edge_list2[i].dir, 0));   // edge

    //  if edgelete is not enought, we select the max grad point in each grid
    n = n - a ;
    if(n>0)
    {

      sort(maxgrad_list2.begin(), maxgrad_list2.end());
      for( int i = 0; i< n; i++)
      {
            fts.push_back(new Feature(frame,maxgrad_list2[i].xy, 0));
      }
    }

  }

   n = num_feature - region_featrue_cnt[2];
  if( n > 0)
  {

    sort(edge_list3.begin(), edge_list3.end());
    int a = (n>edge_list3.size())? edge_list3.size():n;
    for( int i = 0; i< a; i++)
      fts.push_back(new Feature(frame, edge_list3[i].xy, edge_list3[i].dir, 0));   // edge

    //  if edgelete is not enought, we select the max grad point in each grid
    n = n - a ;
    if(n>0)
    {

      sort(maxgrad_list3.begin(), maxgrad_list3.end());
      for( int i = 0; i< n; i++)
      {
            fts.push_back(new Feature(frame,maxgrad_list3[i].xy, 0));
      }
    }

  }

  n = num_feature - region_featrue_cnt[3];
  if( n > 0)
  {

    sort(edge_list4.begin(), edge_list4.end());
    int a = (n>edge_list4.size())? edge_list4.size():n;
    for( int i = 0; i< a; i++)
      fts.push_back(new Feature(frame, edge_list4[i].xy, edge_list4[i].dir, 0));   // edge

    //  if edgelete is not enought, we select the max grad point in each grid
    n = n - a ;
    if(n>0)
    {

      sort(maxgrad_list4.begin(), maxgrad_list4.end());
      for( int i = 0; i< n; i++)
      {
            fts.push_back(new Feature(frame,maxgrad_list4[i].xy, 0));
      }
    }

  }

  resetGrid();

#endif
}
*/

#define EDGE
void EdgeDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
#ifdef EDGE
  setExistingFeatures(fts);  // corners

  cv::Mat img = img_pyr[0].clone();
  cv::Mat gradx = cv::Mat::zeros( img_pyr[0].rows, img_pyr[0].cols, CV_32F);
  cv::Mat grady = cv::Mat::zeros(img_pyr[0].rows, img_pyr[0].cols, CV_32F);
  cv::Mat mag =  cv::Mat::zeros(img_pyr[0].rows, img_pyr[0].cols, CV_32F);

  cv::GaussianBlur( img_pyr[0], img, cv::Size( 3, 3 ), 0, 0 );
  cv::Scharr(img, gradx, CV_32F, 1, 0, 1/32.0);
  cv::Scharr(img, grady, CV_32F, 0, 1, 1/32.0);
  cv::magnitude(gradx,grady,mag);

  vector< Pt > maxgrad_list;
  vector < Edgelete > edge_list;

  for(size_t k = 0; k<grid_occupancy_.size(); k++)
  {

    // if  no fast corner at the cell, we will detect the edge feature or max grad point
    if(grid_occupancy_[k])
    {
      continue;
    }

    int n = std::floor(k/grid_n_cols_);
    // discard the cell at the boundary
    if( n == 0 || n == (grid_n_rows_-1)  || (k%grid_n_cols_) == 0 || ((k+1)%grid_n_cols_ == 0 ))
      continue;

    // calculate the start pixel coordinate  of  k'th cell
    int x_start = (k - n*grid_n_cols_)* cell_size_;
    int y_start = n*cell_size_;

    float max_grad = 0;
    int maxgrad_x = 0;
    int maxgrad_y = 0;
    float gx = 0;
    float gy = 0;

     cv::Mat canny;
     cv::Rect roi(x_start,y_start,cell_size_,cell_size_);
     cv::Canny(img(roi) , canny , 30,50);

    float max_grad_2 = 0;
    int maxgrad_x2 = 0;
    int maxgrad_y2 = 0;

    for (int i=0;i<cell_size_;i++ )
      for(int j=0;j<cell_size_;j++)
    {

        float temp = mag.ptr<float>(y_start + i)[x_start+j] ;
        if( temp> max_grad_2 )
        {
          maxgrad_x2 = x_start+j;
          maxgrad_y2 = y_start + i;
          max_grad_2 = temp;
        }

            if(canny.ptr<uchar>(i)[j] == 0) continue;
            if( temp> max_grad )
            {
              maxgrad_x = x_start+j;
              maxgrad_y = y_start + i;
              max_grad = temp;
              gx = gradx.ptr<float>(maxgrad_y)[maxgrad_x] ;
              gy = grady.ptr<float>(maxgrad_y)[maxgrad_x] ;
            }
    }

    int edge_threshold = 4;
    {

          Vector2d g = Vector2d(gx,gy);
          g.normalize();

          if( max_grad > edge_threshold)
         {
            Edgelete e(max_grad,Vector2d(maxgrad_x, maxgrad_y),g);
            edge_list.push_back(e);
            //fts.push_back(new Feature(frame, Vector2d(maxgrad_x, maxgrad_y),g, 0));   // edge
          }
          else if( max_grad_2 >0.0)
          {
              Pt p(max_grad_2,Vector2d(maxgrad_x2,maxgrad_y2));
              maxgrad_list.push_back(p);
        //     fts.push_back(new Feature(frame, Vector2d(maxgrad_x2,maxgrad_y2), 0));
          }

    }

  }

  int num_feature = 240;
  int n = num_feature - fts.size();
  if( n > 0)
  {

    sort(edge_list.begin(), edge_list.end());
    int a = (n>edge_list.size())? edge_list.size():n;
    for( int i = 0; i< a; i++)
      fts.push_back(new Feature(frame, edge_list[i].xy, edge_list[i].dir, 0));   // edge

    //  if edgelete is not enought, we select the max grad point in each grid
    n = n - a ;
    if(n>0)
    {

      sort(maxgrad_list.begin(), maxgrad_list.end());
      for( int i = 0; i< n; i++)
      {
            fts.push_back(new Feature(frame,maxgrad_list[i].xy, 0));
      }
    }

  }

  resetGrid();

#endif
}

} // namespace feature_detection
} // namespace svo


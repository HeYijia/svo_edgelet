# SVO_edgelet
ChangeList， compare with the origin svo:
1. add edgelete feature
2. inition VO: auto select H model and E model
3. keyframe selection strategy
4. add pangolin to draw trajectory and remove ROS

## install（ubuntu）

### Prerequisites
* boost - c++ Librairies (thread and system are needed)
	> sudo apt-get install libboost-all-dev	
* Eigen 3 - Linear algebra
	> apt-get install libeigen3-dev
* OpenCV3
* Sophus - Lie groups
    we have intergrated sophus in our code. Thanks for the code: https://github.com/strasdat/Sophus.git
 
* g2o - General Graph Optimization **OPTIONAL**	
	Only required if you want to run bundle adjustment. It is not necessary for visual odometry. 
    I suggest an out-of-source build of g2o:
	> cd workspace	
	> 
	> git clone https://github.com/RainerKuemmerle/g2o.git
	> 
	> cd g2o	
	> 
	> mkdir build	
	> 
	> cd build	
	> 
	> cmake ..	
	> 
	> make	
	> 
	> sudo make install

### Build and Compile
If you want use g2o,  please set flag in cmakerlists.txt：
>SET(HAVE_G2O TRUE)   #TRUE  FALSE

compile：
> mkdir build
> cd build
> cmake ..	
> make

## Run code
test file, test_pipline and test_live_vo:

Offline datasets：

	To run on tum dataset：test_pipline
	To run on euroc dataset：test_piplel_euroc

Online camera：test_live_vo

## Results
![Results1](http://img.blog.csdn.net/20170312231721526?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvaGV5aWppYTAzMjc=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)
## Authors
Yijia He, Institute of Automation, Chinese Academy of Sciences

Wei Wei, Institute of Computing Technology, Chinese Academy of Sciences

Yanan Gao, Institute of Computing Technology, Chinese Academy of Sciences

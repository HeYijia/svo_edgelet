# SVO_AR
## 安装程序（ubuntu）
### 安装依赖项
* boost - c++ Librairies (thread and system are needed)
	> sudo apt-get install libboost-all-dev	
* Eigen 3 - Linear algebra
	> apt-get install libeigen3-dev
* Sophus - Lie groups
 	> cd workspace
 > git clone https://github.com/strasdat/Sophus.git	
 > cd Sophus	
 > <font color=red>git checkout a621ff</font>	
 > mkdir build	
 > cd build	
 > cmake ..	
 > make

	You don't need to install the library since cmake .. writes the package location to ~/.cmake/packages/ where CMake can later find it.
* g2o - General Graph Optimization **OPTIONAL**	
	Only required if you want to run bundle adjustment. It is not necessary for visual odometry. 
    I suggest an out-of-source build of g2o:
	> cd workspace	
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o	
mkdir build	
cd build	
cmake ..	
make	
sudo make install

### 编译
下载程序到指定路径，程序默认不使用g2o，如果编译的时候想使用g2o，修改cmakerlists.txt：
>SET(HAVE_G2O TRUE)   #TRUE  FALSE

开始编译：
>mkdir build	
>cmake ..	
>make

## 运行程序
目前可供测试的程序是test文件夹下的test_pipline和test_live_vo。

test_pipline是从文件夹获取图片。

test_live_vo是实时获取摄像头。
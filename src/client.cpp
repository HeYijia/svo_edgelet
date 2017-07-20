#include "svo/client.h"
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include<arpa/inet.h>

#include <string.h>

#include <unistd.h>
#include <iostream>
#include <math.h>

namespace SVO {

Eigen::Matrix3d
rpy2dcm(const Eigen::Vector3d &rpy)
{
  Eigen::Matrix3d R1;
  R1(0,0) = 1.0; R1(0,1) = 0.0; R1(0,2) = 0.0;
  R1(1,0) = 0.0; R1(1,1) = std::cos(rpy[0]); R1(1,2) = -std::sin(rpy[0]);
  R1(2,0) = 0.0; R1(2,1) = -R1(1,2); R1(2,2) = R1(1,1);

  Eigen::Matrix3d R2;

  R2(0,0) = std::cos(rpy[1]); R2(0,1) = 0.0; R2(0,2) = std::sin(rpy[1]);
  R2(1,0) = 0.0; R2(1,1) = 1.0; R2(1,2) = 0.0;
  R2(2,0) = -R2(0,2); R2(2,1) = 0.0; R2(2,2) = R2(0,0);

  Eigen::Matrix3d R3;
  R3(0,0) = std::cos(rpy[2]); R3(0,1) = -std::sin(rpy[2]); R3(0,2) = 0.0;
  R3(1,0) = -R3(0,1); R3(1,1) = R3(0,0); R3(1,2) = 0.0;
  R3(2,0) = 0.0; R3(2,1) = 0.0; R3(2,2) = 1.0;

  return R3 * R2 * R1;
}

SocketClient::SocketClient(std::string hostname, int port):
  hostname_(hostname),port_(port)
{

  T_world_from_vision_ = Sophus::SE3(rpy2dcm(Eigen::Vector3d(3.14,0.0,0.0)),
      Eigen::Vector3d(0.0,0.0,0.0));

  std::cout<<T_world_from_vision_.rotation_matrix();
  close_socket_flag = false;
  sockfd_ = initSock(hostname.c_str(),port);

}

int SocketClient::initSock(const char *ip, int port)
{

    struct sockaddr_in servaddr;
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        printf("create socket error: %s(errno: %d\n)\n", strerror(errno), errno);
        return -1;
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &servaddr.sin_addr) <= 0)
    {
        std::cout<<"inet_pton error for"<< ip<<std::endl;
        return -1;
    }
    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
       std::cout<<"connect error"  <<std::endl;
        return -1;
    }


    return sockfd;
}

int SocketClient::SendPose(Sophus::SE3 T_world_cam, double t)
{

  if(sockfd_ > 0)
  {
    std::stringstream ss;


    //Sophus::SE3 Twc;
    std::cout<<T_world_cam.translation()<<std::endl;

    //Eigen::Vector3d twc = T_world_from_vision_.rotation_matrix() * T_world_cam.translation();
    //Eigen::Matrix3d Rwc = T_world_from_vision_.rotation_matrix() * T_world_cam.rotation_matrix();
    Eigen::Vector3d twc =  T_world_cam.translation();
    Eigen::Matrix3d Rwc = T_world_cam.rotation_matrix();
    std::cout<<twc<<std::endl;

    Eigen::Quaterniond q(Rwc);
    ss<<std::fixed<< t << " "
                     << twc(0) << " "
                     << twc(1) << " "
                     << twc(2) << " "
                     << q.x() << " "
                     << q.y() << " "
                     << q.z() << " "
                     << q.w() << " "
                     << std::endl;

    std::string s = ss.str();
   return  send(sockfd_, s.c_str(), s.size(), 0);
  }
}

}

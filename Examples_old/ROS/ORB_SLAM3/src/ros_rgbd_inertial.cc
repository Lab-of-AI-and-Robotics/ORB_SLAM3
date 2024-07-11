/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGBD_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;

  // Modified for launch files
  std::string node_name = ros::this_node::getName();
  std::string voc_file, settings_file;
  n.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
  n.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

  if (voc_file == "file_not_set" || settings_file == "file_not_set")
  {
      ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
      ros::shutdown();
      return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(voc_file, settings_file,ORB_SLAM3::System::IMU_RGBD,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO

  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(n, "/camera/rgb/image_raw", 100);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(n, "/camera/depth_registered/image_raw", 100);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &RGB_msg, const sensor_msgs::ImageConstPtr &D_msg)
{
  mBufMutex.lock();

  if (!imgRGBBuf.empty())
    imgRGBBuf.pop();
  imgRGBBuf.push(RGB_msg);

  if (!imgDBuf.empty())
      imgDBuf.pop();
  imgDBuf.push(D_msg);

  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
    {
      cv::Mat im, depth;
      double tIm = 0;
      tIm = imgRGBBuf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      
      this->mBufMutex.lock();
      im = GetImage(imgRGBBuf.front());
      imgRGBBuf.pop();
      depth = GetImage(imgDBuf.front());
      imgDBuf.pop();
      this->mBufMutex.unlock();
      

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(im,im);
        mClahe->apply(depth,depth);
      }
      mpSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
    }
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}



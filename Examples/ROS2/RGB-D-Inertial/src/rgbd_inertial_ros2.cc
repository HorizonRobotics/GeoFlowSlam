/**
 * This file is part of ORB-SLAM3
 *
 * Copydepth (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copydepth (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <vector>

#include "../../../include/ImuTypes.h"
#include "../../../include/System.h"

using namespace std;

float shift = 0;

class ImageGrabber : public rclcpp::Node {
 public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe)
      : Node("image_grabber"),
        mpSLAM(pSLAM),
        do_rectify(bRect),
        mbClahe(bClahe) {
    rclcpp::QoS qos_profile(100);  // 默认的 QoS 策略，队列大小为 100
    rclcpp::QoS qos_profile_imu(1000);  // 默认的 QoS 策略，队列大小为 100
    rclcpp::QoS qos_profile_odom(1000);  // 默认的 QoS 策略，队列大小为 100
    qos_profile.reliability(
        rclcpp::ReliabilityPolicy::Reliable);  // 设置为可靠的 QoS 策略
    qos_profile_imu.reliability(
        rclcpp::ReliabilityPolicy::BestEffort);  // 设置为BestEffort QoS 策略
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", qos_profile,
        std::bind(&ImageGrabber::GrabImageRgb, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", qos_profile,
        std::bind(&ImageGrabber::GrabImageDepth, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/imu", qos_profile_imu,
        std::bind(&ImageGrabber::GrabImu, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_odom", qos_profile_odom,
        std::bind(&ImageGrabber::GrabOdom, this, std::placeholders::_1));
  }

  void GrabImageRgb(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBufMutexRgb);
    if (!imgRgbBuf.empty()) imgRgbBuf.pop();
    // cout << "RGB image received" << endl;
    imgRgbBuf.push(msg);
  }

  void GrabImageDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBufMutexDepth);
    if (!imgDepthBuf.empty()) imgDepthBuf.pop();
    // cout << "Depth image received" << endl;
    imgDepthBuf.push(msg);
  }
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
    // cout << "IMU data received" << endl;
  }
  void GrabOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    std::lock_guard<std::mutex> lock(mBufMutexOdom);
    odomBuf.push(odom_msg);
    // cout << "Odom data received" << endl;
  }
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    try {
      return cv_bridge::toCvShare(img_msg)->image.clone();
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
  }

  void SyncWithImu() {
    const double maxTimeDiff = 0.03333;
    while (rclcpp::ok()) {
      cv::Mat imRgb, imDepth;
      double tImRgb = 0, tImDepth = 0;
      // if (imuBuf.empty()) {
      //   // cout << "IMU data is empty" << endl;
      // }
      if (!imgRgbBuf.empty() && !imgDepthBuf.empty() && !imuBuf.empty()) {
        tImRgb = imgRgbBuf.front()->header.stamp.sec +
                 imgRgbBuf.front()->header.stamp.nanosec * 1e-9;
        tImDepth = imgDepthBuf.front()->header.stamp.sec +
                   imgDepthBuf.front()->header.stamp.nanosec * 1e-9;
        // cout << "tImRgb: " << tImRgb << ", tImDepth: " << tImDepth << endl;
        {
          std::lock_guard<std::mutex> lock(mBufMutexDepth);
          while ((tImRgb - tImDepth) > maxTimeDiff && imgDepthBuf.size() > 1) {
            imgDepthBuf.pop();
            tImDepth = imgDepthBuf.front()->header.stamp.sec +
                       imgDepthBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexRgb);
          while ((tImDepth - tImRgb) > maxTimeDiff && imgRgbBuf.size() > 1) {
            imgRgbBuf.pop();
            tImRgb = imgRgbBuf.front()->header.stamp.sec +
                     imgRgbBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        if (std::abs(tImRgb - tImDepth) > maxTimeDiff) continue;
        if (tImRgb > imuBuf.back()->header.stamp.sec +
                         imuBuf.back()->header.stamp.nanosec * 1e-9)
          continue;

        {
          std::lock_guard<std::mutex> lock(mBufMutexRgb);
          imRgb = GetImage(imgRgbBuf.front());
          imgRgbBuf.pop();
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexDepth);
          imDepth = GetImage(imgDepthBuf.front());
          imgDepthBuf.pop();
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mBufMutex);
          while (!imuBuf.empty() &&
                 imuBuf.front()->header.stamp.sec +
                         imuBuf.front()->header.stamp.nanosec * 1e-9 <=
                     tImRgb + shift) {
            double t = imuBuf.front()->header.stamp.sec +
                       imuBuf.front()->header.stamp.nanosec * 1e-9;
            cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
                            imuBuf.front()->linear_acceleration.y,
                            imuBuf.front()->linear_acceleration.z);
            cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
                            imuBuf.front()->angular_velocity.y,
                            imuBuf.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            imuBuf.pop();
          }
        }
        vector<Eigen::Vector3f> vOdomMeas;
        {
          std::lock_guard<std::mutex> lock(mBufMutex);
          while (!odomBuf.empty() &&
                 odomBuf.front()->header.stamp.sec +
                         odomBuf.front()->header.stamp.nanosec * 1e-9 <=
                     tImRgb + shift) {
            double t = odomBuf.front()->header.stamp.sec +
                       odomBuf.front()->header.stamp.nanosec * 1e-9;
            Eigen::Vector3f pos(odomBuf.front()->twist.twist.linear.x,
                                odomBuf.front()->twist.twist.linear.y,
                                odomBuf.front()->twist.twist.linear.z);
            vOdomMeas.push_back(pos);
            odomBuf.pop();
          }
        }

        // if (mbClahe) {
        //   mClahe->apply(imRgb, imRgb);
        //   mClahe->apply(imDepth, imDepth);
        // }

        cv::Size dsize = cv::Size(640, 480);
        cv::Mat rgb_resize, depth_resize;
        cv::resize(imRgb, rgb_resize, dsize, 0, 0, cv::INTER_NEAREST);
        cv::resize(imDepth, depth_resize, dsize, 0, 0, cv::INTER_NEAREST);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 =
            std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 =
            std::chrono::monotonic_clock::now();
#endif
        {
          // std::lock_guard<std::mutex> lock(track_mutex_);
#ifdef ENABLE_ASYNC
          mpSLAM->TrackRGBDAsync(rgb_resize, depth_resize, tImRgb, vImuMeas,vOdomMeas);
#else
          mpSLAM->TrackRGBD(rgb_resize, depth_resize, tImRgb, vImuMeas,vOdomMeas);
#endif
        }
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 =
            std::chrono::monotonic_clock::now();
#endif
        double ttrack =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
                .count();

        double T = 0;
        if (!imgRgbBuf.empty()) {
          double tframe = imgRgbBuf.front()->header.stamp.sec +
                          imgRgbBuf.front()->header.stamp.nanosec * 1e-9;
          T = tframe - tImRgb;
        }

        if (ttrack < T) {
          std::this_thread::sleep_for(
              std::chrono::duration<double>(T - ttrack));
        } else {
          std::chrono::milliseconds tSleep(1);
          std::this_thread::sleep_for(tSleep);
        }
        // std::chrono::milliseconds tSleep(1);
        // std::this_thread::sleep_for(tSleep);
      }
    }
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgRgbBuf, imgDepthBuf;
  std::mutex mBufMutexRgb, mBufMutexDepth;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<nav_msgs::msg::Odometry::SharedPtr> odomBuf;
  std::mutex mBufMutex;
  std::mutex mBufMutexOdom;
  ORB_SLAM3::System *mpSLAM;
  std::mutex track_mutex_;  // 用于同步 TrackRGBD 调用的互斥锁
  // std::shared_ptr<ImuGrabber> mpImuGb;

  const bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: ros2 run ORB_SLAM3 RGBD_inertial path_to_vocabulary "
                 "path_to_settings path_to_save_dir"
              << std::endl;
    return 1;
  }

  bool bEqual = (argc == 5 && std::string(argv[4]) == "true");
  // ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD,
  // true);

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true, 0,
                         std::string(), std::string(argv[3]));
  // auto imu_grabber = std::make_shared<ImuGrabber>();
  auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, false, false);

  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
    return -1;
  }

  std::thread sync_thread(&ImageGrabber::SyncWithImu, image_grabber);

  rclcpp::spin(image_grabber);
  rclcpp::shutdown();
  SLAM.Shutdown(std::string(argv[3]));

  sync_thread.join();

  return 0;
}
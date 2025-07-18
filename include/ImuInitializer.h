/**************************************************************************
 * VIG-Init
 *
 * Copyright SenseTime. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************/

#ifndef IMUINITIALIZER_H
#define IMUINITIALIZER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ImuTypes.h"
#include "KeyFrame.h"

namespace ORB_SLAM3 {

class KeyFrame;

class ImuInitializer {
 public:
  ImuInitializer(std::vector<KeyFrame*>& kfs, Eigen::Vector3f bg);
  ~ImuInitializer();

  bool init_imu();
  void preintegrate();

  void solve_gravity_scale();
  void solve_scale_ba_given_gravity();
  void refine_scale_ba_via_gravity();

  Eigen::Matrix<float, 3, 2> s2_tangential_basis(Eigen::Vector3f& x);

  // void apply_init();

  // IMU bias
  // IMU::Bias imuBias;
  Eigen::Vector3f ba, bg, gravity;
  float scale;
  std::vector<KeyFrame*> kfs;
  std::vector<std::shared_ptr<ORB_SLAM3::IMU::Preintegrated>> imuPres;

  // Eigen::Vector3f gravity;
  // Eigen::Vector3f mVw;
  float GRAVITY_NOMINAL;
};

}  // namespace ORB_SLAM3

#endif

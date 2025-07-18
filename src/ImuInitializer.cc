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

#include "ImuInitializer.h"

#include <bits/stdc++.h>

#include <iostream>

#include "ImuTypes.h"

using namespace std;

namespace ORB_SLAM3 {
using namespace Eigen;

ImuInitializer::ImuInitializer(std::vector<KeyFrame*>& kfs, Eigen::Vector3f bg)
    : kfs(kfs), bg(bg) {
  GRAVITY_NOMINAL = 9.80665;
  ba.setZero();
  gravity.setZero();
  scale = 1.0;
  imuPres.resize(kfs.size());
  for (int i = 0; i < kfs.size(); i++) {
    imuPres[i] = std::make_shared<ORB_SLAM3::IMU::Preintegrated>();
    if (kfs[i]->mpImuPreintegrated)
      imuPres[i]->CopyFrom(kfs[i]->mpImuPreintegrated);
  }
}

ImuInitializer::~ImuInitializer() {};

bool ImuInitializer::init_imu() {
  // check imu observibility
  preintegrate();
  Eigen::Vector3f sum_g;
  float dis = 0.0;
  for (int i = 1; i < imuPres.size(); i++) {
    float dt = imuPres[i]->dT;
    Eigen::Vector3f tmp_g = imuPres[i]->dV / dt;
    sum_g += tmp_g;
    dis += imuPres[i]->dP.norm();
  }

  dis = dis / (imuPres.size() - 1);
  if (dis < 0.02) return false;

  Eigen::Vector3f aver_g;
  aver_g = sum_g * 1.0 / ((int)imuPres.size() - 1);
  float var = 0;
  for (int i = 1; i < imuPres.size(); i++) {
    float dt = imuPres[i]->dT;
    Eigen::Vector3f tmp_g = imuPres[i]->dV / dt;
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
  }

  var = sqrt(var / ((int)imuPres.size() - 1));
  if (var < 0.4) {
    return false;
  }

  solve_gravity_scale();
  solve_scale_ba_given_gravity();
  refine_scale_ba_via_gravity();

  std::cout << std::fixed << std::setprecision(5) << "scale=" << scale
            << ", bw=" << bg.transpose() << ", ba=" << ba.transpose()
            << ", gravity=" << gravity.transpose() << std::endl;

  return true;
}

// IMU::Bias b(0,0,0,0,0,0); frames[i].mpImuPreintegratedFrame->Initialize(b);
void ImuInitializer::preintegrate() {
  IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
  for (int i = 1; i < imuPres.size(); i++) {
    imuPres[i]->SetNewBias(b);
    imuPres[i]->Reintegrate();
  }
}

void ImuInitializer::solve_gravity_scale() {
  Eigen::Matrix4f A;
  Eigen::Vector4f b;
  A.setZero();
  b.setZero();

  for (size_t i = 2; i < imuPres.size(); ++i) {
    ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i - 1].get();
    ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

    Sophus::SE3f Twb1 = kfs[i - 2]->GetImuPose();
    Sophus::SE3f Twb2 = kfs[i - 1]->GetImuPose();
    Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

    Matrix<float, 3, 4> C;
    C.block<3, 3>(0, 0) = -0.5 * pInt12->dT * pInt23->dT *
                          (pInt12->dT + pInt23->dT) * Matrix3f::Identity();
    C.block<3, 1>(0, 3) =
        pInt12->dT * (Twb3.translation() - Twb2.translation()) -
        pInt23->dT * (Twb2.translation() - Twb1.translation());
    Vector3f d =
        pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) +
        pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) -
        pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  JacobiSVD<Matrix4f> svd(A, ComputeFullU | ComputeFullV);
  Vector4f x = svd.solve(b);
  gravity = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
  scale = x(3);
}

void ImuInitializer::solve_scale_ba_given_gravity() {
  Eigen::Matrix4f A;
  Eigen::Vector4f b;
  A.setZero();
  b.setZero();

  for (size_t i = 2; i < imuPres.size(); ++i) {
    ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i - 1].get();
    ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

    Sophus::SE3f Twb1 = kfs[i - 2]->GetImuPose();
    Sophus::SE3f Twb2 = kfs[i - 1]->GetImuPose();
    Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

    Matrix<float, 3, 4> C;
    // delta_jk

    C.block<3, 1>(0, 0) =
        pInt12->dT * (Twb3.translation() - Twb2.translation()) -
        pInt23->dT * (Twb2.translation() - Twb1.translation());
    C.block<3, 3>(0, 1) =
        -(Twb2.rotationMatrix() * pInt23->JPa * pInt12->dT +
          Twb1.rotationMatrix() * pInt12->JVa * pInt12->dT * pInt23->dT -
          Twb1.rotationMatrix() * pInt12->JPa * pInt23->dT);
    Vector3f d =
        0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * gravity +
        pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) +
        pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) -
        pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  JacobiSVD<Matrix4f> svd(A, ComputeFullU | ComputeFullV);
  Vector4f x = svd.solve(b);
  scale = x(0);
  ba = x.segment<3>(1);
}

void ImuInitializer::refine_scale_ba_via_gravity() {
  static const float damp = 0.1;
  Matrix<float, 6, 6> A;
  Matrix<float, 6, 1> b;
  for (size_t iter = 0; iter < 3; ++iter) {
    A.setZero();
    b.setZero();
    Matrix<float, 3, 2> Tg = s2_tangential_basis(gravity);

    preintegrate();
    for (size_t i = 2; i < imuPres.size(); ++i) {
      ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i - 1].get();
      ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

      Sophus::SE3f Twb1 = kfs[i - 2]->GetImuPose();
      Sophus::SE3f Twb2 = kfs[i - 1]->GetImuPose();
      Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

      Matrix<float, 3, 6> C;
      C.block<3, 1>(0, 0) =
          pInt12->dT * (Twb3.translation() - Twb2.translation()) -
          pInt23->dT * (Twb2.translation() - Twb1.translation());
      C.block<3, 3>(0, 1) =
          -(Twb2.rotationMatrix() * pInt23->JPa * pInt12->dT +
            Twb1.rotationMatrix() * pInt12->JVa * pInt12->dT * pInt23->dT -
            Twb1.rotationMatrix() * pInt12->JPa * pInt23->dT);
      C.block<3, 2>(0, 4) =
          -0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * Tg;
      Vector3f d =
          0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * gravity +
          pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) +
          pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) -
          pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
      A += C.transpose() * C;
      b += C.transpose() * d;
    }

    JacobiSVD<Matrix<float, 6, 6>> svd(A, ComputeFullU | ComputeFullV);
    Matrix<float, 6, 1> x = svd.solve(b);
    scale = x(0);
    ba += damp * x.segment<3>(1);
    gravity =
        (gravity + damp * Tg * x.segment<2>(4)).normalized() * GRAVITY_NOMINAL;
  }
}

Eigen::Matrix<float, 3, 2> ImuInitializer::s2_tangential_basis(
    Eigen::Vector3f& x) {
  int d = 0;
  for (int i = 1; i < 3; ++i) {
    if (abs(x[i]) > abs(x[d])) d = i;
  }
  Eigen::Vector3f b1 = x.cross(Eigen::Vector3f::Unit((d + 1) % 3)).normalized();
  Eigen::Vector3f b2 = x.cross(b1).normalized();
  return (Eigen::Matrix<float, 3, 2>() << b1, b2).finished();
}

}  // namespace ORB_SLAM3

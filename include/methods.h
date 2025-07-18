
#ifndef METHODS_H_
#define METHODS_H_

// STL
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/se3.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

// Ceres
#include <ceres/ceres.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Glog
#include <glog/logging.h>

#include "ImuTypes.h"
#include "imu_ceres.h"
#include "polynomial.h"
#include "svd.h"

/*
struct input_t {
  input_t(const Eigen::Isometry3d &T1, const std::uint64_t t1,
          const Eigen::Isometry3d &T2, const std::uint64_t t2,
          std::shared_ptr<ORB_SLAM3::IMU::Preintegrated> pInt)
    : t1(t1), t2(t2), T1(T1), T2(T2), pInt(pInt)
  { }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const std::uint64_t t1, t2;
  Eigen::Isometry3d T1, T2;
  std::shared_ptr<ORB_SLAM3::IMU::Preintegrated> pInt;

  std::vector<ORB_SLAM3::IMU::Measurement> vMeasurements;
};
*/

struct result_t {
  result_t() : success(false) {}

  result_t(bool success, std::int64_t solve_ns, double scale,
           const Eigen::Vector3d& bias_g, const Eigen::Vector3d& bias_a,
           const Eigen::Vector3d& gravity)
      : success(success),
        solve_ns(solve_ns),
        scale(scale),
        bias_g(bias_g),
        bias_a(bias_a),
        gravity(gravity) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool success;
  std::int64_t solve_ns, velocities_ns;
  double scale;
  Eigen::Vector3d bias_g, bias_a, gravity;
};

// using InputType = std::vector<input_t>;
using ResultType = result_t;

void gyroscope_only(const std::vector<ORB_SLAM3::KeyFrame*>& input,
                    ResultType& result, bool use_covarinace = true) {
  double** parameters = new double*[1];
  parameters[0] = new double[3];
  Eigen::Map<Eigen::Vector3d> bias_(parameters[0]);
  bias_.setZero();

  ceres::Problem problem;
  for (unsigned i = 1; i < input.size(); ++i) {
    Eigen::Matrix3d R1 = input[i - 1]->GetImuRotation().cast<double>();
    Eigen::Matrix3d R2 = input[i]->GetImuRotation().cast<double>();

    ORB_SLAM3::IMU::Preintegrated* pInt = input[i]->mpImuPreintegrated;
    ceres::CostFunction* cost_function =
        new GyroscopeBiasCostFunction(pInt, R1, R2, use_covarinace);
    problem.AddResidualBlock(cost_function, nullptr, parameters[0]);  //, 1);
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  bool converged = (summary.termination_type == ceres::CONVERGENCE);
  if (converged) {
    result.success = true;
    result.bias_g = bias_;
  } else {
    std::cout << summary.FullReport() << std::endl;
    result.success = false;
  }

  delete[] parameters[0];
  delete[] parameters;
}

Eigen::VectorXd real_roots(const Eigen::VectorXd& real,
                           const Eigen::VectorXd& imag) {
  CHECK_EQ(real.size(), imag.size());

  Eigen::VectorXd roots(real.size());

  Eigen::VectorXd::Index j = 0;
  for (Eigen::VectorXd::Index i = 0; i < real.size(); ++i) {
    if (!imag(i)) {
      roots(j) = real(i);
      ++j;
    }
  }

  roots.conservativeResize(j);
  return roots;
}

void analytic_accelerometer(const std::vector<ORB_SLAM3::KeyFrame*>& input,
                            ResultType& result,
                            const Eigen::Vector3f& bg = Eigen::Vector3f::Zero(),
                            const Eigen::Vector3f& ba = Eigen::Vector3f::Zero(),
                            const double prior = 0.0) {
  // std::cout << "Running proposed method at " << input[0].t1 << std::endl;

  // CHECK_GE(prior, 0.0);

  constexpr int n = 7;
  constexpr int q = 4;

  Eigen::MatrixXf M(n, n);
  M.setZero();

  Eigen::VectorXf m(n);
  m.setZero();

  double Q = 0.;

  const Eigen::Vector3f ba_prior_mean = Eigen::Vector3f::Zero();

  // accelerometer bias prior
  {
    Eigen::MatrixXf M_k(3, n);
    M_k.setZero();

    M_k.block<3, 3>(0, 1) = Eigen::Matrix3f::Identity();

    Eigen::Vector3f pi_k;
    pi_k = ba_prior_mean;

    Eigen::Matrix3f Information = prior * Eigen::Matrix3f::Identity();

    M += M_k.transpose() * Information * M_k;
    m += -2. * M_k.transpose() * Information * pi_k;
    Q += pi_k.transpose() * Information * pi_k;
  }

  // for (unsigned i = 1; i < input.size(); ++i) {
  for (int i = 2; i < input.size(); i++) {
    // CHECK_EQ(input[i-1].t2, input[i].t1);
    Sophus::SE3f Twc1 = input[i - 2]->GetPoseInverse();
    Sophus::SE3f Twc2 = input[i - 1]->GetPoseInverse();
    Sophus::SE3f Twc3 = input[i]->GetPoseInverse();

    ORB_SLAM3::IMU::Preintegrated* pInt12 = input[i - 1]->mpImuPreintegrated;
    ORB_SLAM3::IMU::Preintegrated* pInt23 = input[i]->mpImuPreintegrated;

    Eigen::Matrix3f Rwb1 = input[i - 2]->GetImuRotation();
    Eigen::Matrix3f Rwb2 = input[i - 1]->GetImuRotation();

    Eigen::Matrix3f A = Rwb1 / pInt12->dT;
    Eigen::Matrix3f B = Rwb2 / pInt23->dT;

    Eigen::MatrixXf M_k(3, n);
    M_k.setZero();
    // translation
    M_k.col(0) = (Twc3.translation() - Twc2.translation()) / pInt23->dT -
                 (Twc2.translation() - Twc1.translation()) / pInt12->dT;

    M_k.block<3, 3>(0, 1) =
        A * pInt12->JPa - B * pInt23->JPa - Rwb1 * pInt12->JVa;
    M_k.block<3, 3>(0, q) =
        -0.5 * (pInt12->dT + pInt23->dT) * Eigen::Matrix3f::Identity();

    Eigen::Vector3f pi_k;
    ORB_SLAM3::IMU::Bias b_(ba(0), ba(1), ba(2), bg(0), bg(1), bg(2));

    pi_k = B * pInt23->GetDeltaPosition(b_) - A * pInt12->GetDeltaPosition(b_) +
           Rwb1 * pInt12->GetDeltaVelocity(b_) +
           (Twc2.rotationMatrix() - Twc1.rotationMatrix()) *
               input[i]->mImuCalib.mTcb.translation() / pInt12->dT -
           (Twc3.rotationMatrix() - Twc2.rotationMatrix()) *
               input[i]->mImuCalib.mTcb.translation() / pInt23->dT;

    Eigen::Matrix3f Covariance;
    Covariance = A * pInt12->C.block<3, 3>(6, 6) * A.transpose();
    Covariance += B * pInt23->C.block<3, 3>(6, 6) * B.transpose();
    Covariance += Twc1.rotationMatrix() * pInt12->C.block<3, 3>(3, 3) *
                  Twc1.rotationMatrix().transpose();

    Eigen::Matrix3f Information = selfAdjointInverse(Covariance);
    // Eigen::Matrix3d Information = Eigen::Matrix3d::Identity();

    M += M_k.transpose() * Information * M_k;
    m += -2. * M_k.transpose() * Information * pi_k;
    Q += pi_k.transpose() * Information * pi_k;
  }

  // Solve
  Eigen::Matrix4f A = 2. * M.block<4, 4>(0, 0);
  // std::cout << StringPrintf("A: %.16f", A) << std::endl;

  // TODO Check if A is invertible!!
  // Eigen::Matrix3d A_ = A.block<3, 3>(1, 1);
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> svdA_(A_,
  // Eigen::EigenvaluesOnly); result.svA_ = svdA_.eigenvalues(); result.detA_
  // = A_.determinant();

  Eigen::MatrixXf Bt = 2. * M.block<3, 4>(q, 0);
  Eigen::MatrixXf BtAi = Bt * A.inverse();

  Eigen::Matrix3f D = 2. * M.block<3, 3>(q, q);
  Eigen::Matrix3f S = D - BtAi * Bt.transpose();

  // TODO Check if S is invertible!
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> svdS(S,
  // Eigen::EigenvaluesOnly); result.svS = svdS.eigenvalues(); result.detS =
  // S.determinant(); std::cout << StringPrintf("det(S): %.16f",
  // S.determinant()) << std::endl; std::cout << StringPrintf("eigenvalues(S):
  // %.16f %.16f %.16f",
  //                          c[0], svd.eigenvalues()[1],
  //                          svd.eigenvalues()[2]) << std::endl;

  Eigen::Matrix3f Sa = S.determinant() * S.inverse();
  Eigen::Matrix3f U = S.trace() * Eigen::Matrix3f::Identity() - S;

  Eigen::Vector3f v1 = BtAi * m.head<q>();
  Eigen::Vector3f m2 = m.tail<3>();

  Eigen::Matrix3f X;
  Eigen::Vector3f Xm2;

  // X = I
  const float c4 = 16. * (v1.dot(v1) - 2. * v1.dot(m2) + m2.dot(m2));

  X = U;
  Xm2 = X * m2;
  const float c3 = 16. * (v1.dot(X * v1) - 2. * v1.dot(Xm2) + m2.dot(Xm2));

  X = 2. * Sa + U * U;
  Xm2 = X * m2;
  const float c2 = 4. * (v1.dot(X * v1) - 2. * v1.dot(Xm2) + m2.dot(Xm2));

  X = Sa * U + U * Sa;
  Xm2 = X * m2;
  const float c1 = 2. * (v1.dot(X * v1) - 2. * v1.dot(Xm2) + m2.dot(Xm2));

  X = Sa * Sa;
  Xm2 = X * m2;
  const float c0 = (v1.dot(X * v1) - 2. * v1.dot(Xm2) + m2.dot(Xm2));

  const float s00 = S(0, 0), s01 = S(0, 1), s02 = S(0, 2);
  const float s11 = S(1, 1), s12 = S(1, 2), s22 = S(2, 2);

  const float t1 = s00 + s11 + s22;
  const float t2 = s00 * s11 + s00 * s22 + s11 * s22 - std::pow(s01, 2) -
                   std::pow(s02, 2) - std::pow(s12, 2);
  const float t3 = s00 * s11 * s22 + 2. * s01 * s02 * s12 -
                   s00 * std::pow(s12, 2) - s11 * std::pow(s02, 2) -
                   s22 * std::pow(s01, 2);

  Eigen::VectorXf coeffs(7);
  coeffs << 64., 64. * t1, 16. * (std::pow(t1, 2) + 2. * t2),
      16. * (t1 * t2 + t3), 4. * (std::pow(t2, 2) + 2. * t1 * t3), 4. * t3 * t2,
      std::pow(t3, 2);

  const float G2i = 1. / std::pow(ORB_SLAM3::IMU::GRAVITY_MAGNITUDE, 2);

  coeffs(2) -= c4 * G2i;
  coeffs(3) -= c3 * G2i;
  coeffs(4) -= c2 * G2i;
  coeffs(5) -= c1 * G2i;
  coeffs(6) -= c0 * G2i;

  Eigen::VectorXd real, imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs.cast<double>(), &real,
                                          &imag)) {
    std::cout << "Failed to find roots, coeff=" << coeffs.transpose()
              << std::endl;
    //<< StringPrintf("%.16f %.16f %.16f %.16f %.16f %.16f %.16f",
    //               coeffs[0], coeffs[1], coeffs[2], coeffs[3],
    //              coeffs[4], coeffs[5], coeffs[6]) << std::endl;
    result.success = false;
    return;
  }

  Eigen::VectorXd lambdas = real_roots(real, imag);
  if (lambdas.size() == 0) {
    std::cout << "No real roots found" << std::endl;
    //<< StringPrintf("%.16f %.16f %.16f %.16f %.16f %.16f %.16f",
    //               coeffs[0], coeffs[1], coeffs[2], coeffs[3],
    //              coeffs[4], coeffs[5], coeffs[6]) << std::endl;
    result.success = false;
    return;
  }

  Eigen::MatrixXf W(n, n);
  W.setZero();
  W.block<3, 3>(q, q) = Eigen::Matrix3f::Identity();

  Eigen::VectorXf solution;
  double min_cost = std::numeric_limits<double>::max();
  for (Eigen::VectorXf::Index i = 0; i < lambdas.size(); ++i) {
    float lambda = lambdas(i);

    Eigen::FullPivLU<Eigen::MatrixXf> lu(2. * M + 2. * lambda * W);
    Eigen::VectorXf x_ = -lu.inverse() * m;

    float cost = x_.transpose() * M * x_;
    cost += m.transpose() * x_;
    cost += Q;

    if (cost < min_cost) {
      solution = x_;
      min_cost = cost;
    }
  }

  float constraint = solution.transpose() * W * solution;
  if (solution[0] < 1e-3 || constraint < 0. ||
      std::abs(std::sqrt(constraint) - ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) /
              ORB_SLAM3::IMU::GRAVITY_MAGNITUDE >
          0.1) {  // 1e-3) { // TODO
    std::cout << "Discarding bad solution, " << "scale=" << solution[0]
              << ", constraint=" << constraint << ", constraint error="
              << std::abs(std::sqrt(constraint) -
                          ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) /
                     ORB_SLAM3::IMU::GRAVITY_MAGNITUDE
              << std::endl;
    result.success = false;
    return;
  }

  std::cout << "solution, " << "scale=" << solution[0]
            << ", constraint=" << constraint << ", constraint error="
            << std::abs(std::sqrt(constraint) -
                        ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) /
                   ORB_SLAM3::IMU::GRAVITY_MAGNITUDE
            << std::endl;

  result.success = true;
  result.scale = solution[0];
  result.bias_a = solution.segment<3>(1).cast<double>();
  result.gravity = solution.segment<3>(4).cast<double>();

  const int N = input.size() + 1;
  Eigen::VectorXf velocities(3 * N);

  /*
  // Recover velocities
  for (unsigned int i = 0; i < input.size(); ++i) {
    const Eigen::Isometry3d &T1 = input[i].T1;
    const Eigen::Isometry3d &T2 = input[i].T2;
    const IMU::Preintegrated &pInt12 = *(input[i].pInt);

    // -[(g*dt12^2)/2 + R1*dP12 + R1_c*pcb - R2_c*pcb + p1_c*s - p2_c*s]/dt12
    velocities.segment<3>(3*i) = (-0.5*result.gravity*std::pow(pInt12.dT, 2)
                                  -
  T1.linear()*Tcb.linear()*pInt12.GetDeltaPosition(bg, result.bias_a)
                                  - (T1.linear() -
  T2.linear())*Tcb.translation()
                                  - (T1.translation() -
  T2.translation())*result.scale)/pInt12.dT;
  }

  // Lask keyframe velocity
  {
    const Eigen::Isometry3d &T1 = input.back().T2;
    const IMU::Preintegrated &pInt12 = *(input.back().pInt);

    velocities.tail<3>() = velocities.segment<3>(3*(N-2)) +
  result.gravity*pInt12.dT
                           +
  T1.linear()*Tcb.linear()*pInt12.GetDeltaVelocity(bg, result.bias_a);
  }

  */
}

void mqh_accelerometer(const std::vector<ORB_SLAM3::KeyFrame*>& input,
                       ResultType& result,
                       const Eigen::Vector3f& bg = Eigen::Vector3f::Zero()) {
  // L. Huang, S. Pan, S. Wang, P. Zeng and F. Ye, "A fast initialization
  // method of Visual-Inertial Odometry
  //  based on monocular camera," 2018 Ubiquitous Positioning, Indoor
  //  Navigation and Location-Based Services (UPINLBS), 2018, pp. 1-5,
  //  doi: 10.1109/UPINLBS.2018.8559929.

  double scale;

  Eigen::Vector3f ba;
  ba.setZero();

  Eigen::Vector3f gravity;

  // Gravity Approximation
  // ---------------------

  // R. Mur-Artal and J. D. Tardós, "Visual-Inertial Monocular SLAM With Map
  // Reuse,"
  //  in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 796-803,
  //  April 2017, doi: 10.1109/LRA.2017.2653359.

  const int N = input.size();  // number of keyframes

  // Initialize gravity
  {
    const int n = 4;
    Eigen::VectorXf x_(n);

    // Build linear system
    Eigen::MatrixXf A(3 * (N - 2), n);
    Eigen::VectorXf b(3 * (N - 2));
    for (int i = 2; i < input.size(); i++) {
      Sophus::SE3f Twc1 = input[i - 2]->GetPoseInverse();
      Sophus::SE3f Twc2 = input[i - 1]->GetPoseInverse();
      Sophus::SE3f Twc3 = input[i]->GetPoseInverse();

      ORB_SLAM3::IMU::Preintegrated* pInt12 = input[i - 1]->mpImuPreintegrated;
      ORB_SLAM3::IMU::Preintegrated* pInt23 = input[i]->mpImuPreintegrated;

      Eigen::Matrix3f Rwb1 = input[i - 2]->GetImuRotation();
      Eigen::Matrix3f Rwb2 = input[i - 1]->GetImuRotation();

      Eigen::Matrix3f A_ = Rwb1 / pInt12->dT;
      Eigen::Matrix3f B_ = Rwb2 / pInt23->dT;

      A.block<3, 1>(3 * i, 0) =
          (Twc3.translation() - Twc2.translation()) / pInt23->dT -
          (Twc2.translation() - Twc1.translation()) / pInt12->dT;

      // beta_i
      A.block<3, 3>(3 * i, 1) =
          -0.5 * (pInt12->dT + pInt23->dT) * Eigen::Matrix3f::Identity();

      ORB_SLAM3::IMU::Bias b_(0.0, 0.0, 0.0, bg(0), bg(1), bg(2));
      b.segment<3>(3 * i) =
          B_ * pInt23->GetDeltaPosition(b_) -
          A_ * pInt12->GetDeltaPosition(b_) +
          Rwb1 * pInt12->GetDeltaVelocity(b_) +
          (Twc2.rotationMatrix() - Twc1.rotationMatrix()) *
              input[i]->mImuCalib.mTcb.translation() / pInt12->dT -
          (Twc3.rotationMatrix() - Twc2.rotationMatrix()) *
              input[i]->mImuCalib.mTcb.translation() / pInt23->dT;
    }

    // Solution vector
    // x_ = pseudoInverse(A, 1e-6)*b;
    x_ = A.fullPivHouseholderQr().solve(b);

    // Initial gravity solution
    gravity = x_.tail<3>();
  }

  // Refine Gravity and Accelerometer Bias Estimation
  // ------------------------------------------------

  // R. Mur-Artal and J. D. Tardós, "Visual-Inertial Monocular SLAM With Map
  // Reuse,"
  //  in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 796-803,
  //  April 2017, doi: 10.1109/LRA.2017.2653359.

  // Initialize Rwg estimate
  Eigen::Vector3f dirG = gravity.tail<3>();
  const Eigen::Vector3f gI(
      0.0f, 0.0f,
      -1.0f);  // = ORB_SLAM3::IMU::GRAVITY_VECTOR.normalized().cast<float>();
  Eigen::Vector3f v = gI.cross(dirG);
  float theta = std::atan2(v.norm(), gI.dot(dirG));
  if (theta == 0.0 || v.norm() == 0.0 || v.normalized().array().isNaN()[0]) {
    result.success = false;
    return;
  }
  std::cout << "v.normalized()=" << v.normalized().transpose()
            << ", theta=" << theta << std::endl;

  Eigen::Matrix3f Rwi = Sophus::SO3f::exp(v.normalized() * theta)
                            .matrix();  //.matrix().cast<double>();

  Eigen::Matrix3f gI_skew;
  gI_skew << 0.0, -gI(2), gI(1), gI(2), 0.0, -gI(0), -gI(1), gI(0), 0.0;
  // gI_skew << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;

  // Refinement
  {
    const int n = 6;
    Eigen::VectorXf x_(n);

    Eigen::MatrixXf A(3 * (N - 2), n);
    Eigen::VectorXf b(3 * (N - 2));
    for (int i = 2; i < input.size(); ++i) {
      Sophus::SE3f Twc1 = input[i - 2]->GetPoseInverse();
      Sophus::SE3f Twc2 = input[i - 1]->GetPoseInverse();
      Sophus::SE3f Twc3 = input[i]->GetPoseInverse();

      ORB_SLAM3::IMU::Preintegrated* pInt12 = input[i - 1]->mpImuPreintegrated;
      ORB_SLAM3::IMU::Preintegrated* pInt23 = input[i]->mpImuPreintegrated;

      Eigen::Matrix3f Rwb1 = input[i - 2]->GetImuRotation();
      Eigen::Matrix3f Rwb2 = input[i - 1]->GetImuRotation();

      Eigen::Matrix3f A_ = Rwb1 / pInt12->dT;
      Eigen::Matrix3f B_ = Rwb2 / pInt23->dT;

      A.block<3, 1>(3 * i, 0) =
          (Twc3.translation() - Twc2.translation()) / pInt23->dT -
          (Twc2.translation() - Twc1.translation()) / pInt12->dT;

      // phi_i
      A.block<3, 2>(3 * i, 1) =
          0.5 * ((float)ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) *
          (pInt12->dT + pInt23->dT) * (Rwi * gI_skew).topLeftCorner<3, 2>();

      // xi_i
      A.block<3, 3>(3 * i, 3) =
          A_ * pInt12->JPa - B_ * pInt23->JPa - Rwb1 * pInt12->JVa;

      ORB_SLAM3::IMU::Bias b_(0.0, 0.0, 0.0, bg(0), bg(1), bg(2));
      b.segment<3>(3 * i) =
          B_ * pInt23->GetDeltaPosition(b_) -
          A_ * pInt12->GetDeltaPosition(b_) +
          Rwb1 * pInt12->GetDeltaVelocity(b_) +
          (Twc2.rotationMatrix() - Twc1.rotationMatrix()) *
              input[i]->mImuCalib.mTcb.translation() / pInt12->dT -
          (Twc3.rotationMatrix() - Twc2.rotationMatrix()) *
              input[i]->mImuCalib.mTcb.translation() / pInt23->dT +
          0.5 * ((float)ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) *
              (pInt12->dT + pInt23->dT) * Rwi * gI;
    }

    // Solution vector
    x_ = pseudoInverse(A, 1e-6) * b;
    scale = x_(0);
    ba = x_.tail<3>();

    // Refine gravity
    Eigen::Vector3f dTheta;
    dTheta.head<2>() = x_.segment<2>(1);
    dTheta(2) = 0.;

    gravity = ((float)ORB_SLAM3::IMU::GRAVITY_MAGNITUDE) * Rwi *
              (gI - gI_skew * dTheta);
  }

  // The original paper proposes to use the algorithm below to compute scale
  // and velocity. However, according to some benchmarking:
  //  1. It has quadratic complexity
  //  2. The scale factor is less accurate than Mur-Artal's method
  // So we use the original Mur-Artal's formulation for evaluation (the
  // accuracy of the velocities is not tested)

  /*
  Eigen::VectorXf velocities(3*N);

  // Recover velocities
  for (unsigned int i = 0; i < input.size(); ++i) {
    const Eigen::Isometry3d &T1 = input[i].T1;
    const Eigen::Isometry3d &T2 = input[i].T2;
    const IMU::Preintegrated &pInt12 = *(input[i].pInt);

    // -[(g*dt12^2)/2 + R1*dP12 + R1_c*pcb - R2_c*pcb + p1_c*s - p2_c*s]/dt12
    velocities.segment<3>(3*i) = (-0.5*result.gravity*std::pow(pInt12.dT, 2)
                                  -
  T1.linear()*Tcb.linear()*pInt12.GetDeltaPosition(bg, result.bias_a)
                                  - (T1.linear() -
  T2.linear())*Tcb.translation()
                                  - (T1.translation() -
  T2.translation())*result.scale)/pInt12.dT;
  }

  // Lask keyframe velocity
  {
    const Eigen::Isometry3d &T1 = input.back().T2;
    const IMU::Preintegrated &pInt12 = *(input.back().pInt);

    velocities.tail<3>() = velocities.segment<3>(3*(N-2)) +
  result.gravity*pInt12.dT
                           +
  T1.linear()*Tcb.linear()*pInt12.GetDeltaVelocity(bg, result.bias_a);
  }

  */

  // Scale and Velocity Estimation
  // -----------------------------

  // T. Qin and S. Shen, "Robust initialization of monocular visual-inertial
  // estimation on aerial robots,"
  //  2017 IEEE/RSJ International Conference on Intelligent Robots and Systems
  //  (IROS), 2017, pp. 4225-4232, doi: 10.1109/IROS.2017.8206284.

  /*
    {
      const int n = 3*(N+1) + 1;
      Eigen::VectorXd x_(n);

      // Since Huang et. al. doesn't say how the linear system should be
    solved, we use Qin and Shen approach,
      // from VINS-Mono. Source code:
    https://github.com/HKUST-Aerial-Robotics/VINS-Mono
      // File: vins_estimator/src/initial/initial_aligment.cpp
      Eigen::MatrixXd A(n, n);
      A.setZero();

      Eigen::VectorXd b(n);
      b.setZero();

      for (unsigned i = 0; i < input.size(); ++i) {
        const Eigen::Isometry3d &T1 = input[i].T1;   // camera to world
        const Eigen::Isometry3d &T2 = input[i].T2;  // camera to world
        const IMU::Preintegrated &pInt12 = *(input[i].pInt);

        Eigen::Matrix3d R1 = T1.linear()*Tcb.linear(); // R1wb

        // H:
        // [ -dt12, 0, -dt12^2/2, p2_c - p1_c]
        // [    -1, 1,     -dt12,           0]

        // z:
        //  R1*dP12 + R1_c*pcb - R2_c*pcb
        //                        R1*dV12

        Eigen::MatrixXd tmp_A(6, 10);
        tmp_A.setZero();

        tmp_A.block<3, 3>(0, 0) = -pInt12.dT*Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 3).setZero();
        tmp_A.block<3, 3>(0, 6) = -0.5*std::pow(pInt12.dT,
    2)*Eigen::Matrix3d::Identity(); tmp_A.block<3, 1>(0, 9) =
    (T2.translation() - T1.translation()) / 100.0;

        tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 6) = -pInt12.dT*Eigen::Matrix3d::Identity();
        tmp_A.block<3, 1>(3, 9).setZero();

        Eigen::VectorXd tmp_b(6);
        tmp_b.block<3, 1>(0, 0) = R1*pInt12.GetDeltaPosition(bg, ba) +
    (T1.linear() - T2.linear())*Tcb.translation(); tmp_b.block<3, 1>(3, 0) =
    R1*pInt12.GetDeltaVelocity(bg, ba);

        Eigen::MatrixXd r_A = tmp_A.transpose()*tmp_A;
        Eigen::VectorXd r_b = tmp_A.transpose()*tmp_b;

        A.block<6, 6>(3*i, 3*i) += r_A.topLeftCorner<6, 6>();
        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        A.block<6, 4>(3*i, n - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n - 4, 3*i) += r_A.bottomLeftCorner<4, 6>();

        b.segment<6>(3*i) += r_b.head<6>();
        b.tail<4>() += r_b.tail<4>();
      }

      // Solution vector
      A = A * 1000.0;
      b = b * 1000.0;
      x_ = A.ldlt().solve(b);
      //scale = x_(n - 1) / 100.0;
      //gravity = x_.segment<3>(n - 4);
    }
  */

  if (scale < 0.) {
    result.success = false;
    return;
  }

  result.success = true;
  result.scale = scale;
  result.bias_a = ba.cast<double>();
  result.gravity = gravity.cast<double>();
}

#endif  // METHODS_H_

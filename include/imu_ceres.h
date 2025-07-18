
#ifndef IMU_CERES_H
#define IMU_CERES_H

#include <ceres/local_parameterization.h>
#include <ceres/sized_cost_function.h>

#include <cmath>
#include <memory>

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "eigen_defs.h"

// TODO Shouldn't the covariance matrix be (-invJr)*C*(-invJr)T ?
//      where C := pInt->C.block<3, 3>(0, 0)
//      and invJr := InverseRightJacobianSO3(er)
class GyroscopeBiasCostFunction : public ceres::SizedCostFunction<3, 3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GyroscopeBiasCostFunction(ORB_SLAM3::IMU::Preintegrated* pInt,
                            const Eigen::Matrix3d& Ri,
                            const Eigen::Matrix3d& Rj,
                            bool use_covarinace = true)
      : pInt(pInt), Ri(Ri), Rj(Rj) {
    if (use_covarinace) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
          pInt->C.block<3, 3>(0, 0).cast<double>());
      SqrtInformation = solver.operatorInverseSqrt();
    } else
      SqrtInformation = Eigen::Matrix3d::Identity();
  }
  virtual ~GyroscopeBiasCostFunction() {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    Eigen::Map<const Eigen::Vector3d> bg(parameters[0]);

    const Eigen::Matrix3d eR =
        pInt->GetDeltaRotation(bg.cast<float>()).transpose().cast<double>() *
        Ri.transpose() * Rj;
    const Eigen::Vector3d er = ORB_SLAM3::LogSO3(eR);

    Eigen::Map<Eigen::Vector3d> e(residuals);
    e = er;
    e = SqrtInformation * e;

    if (jacobians != nullptr) {
      if (jacobians[0] != nullptr) {
        // wrt gyro bias
        const Eigen::Vector3d dbg = pInt->GetGyroDeltaBias(bg);
        const Eigen::Matrix3d invJr =
            ORB_SLAM3::IMU::InverseRightJacobianSO3(er.cast<float>())
                .cast<double>();

        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(
            jacobians[0]);
        J = -invJr * eR.transpose() *
            ORB_SLAM3::IMU::RightJacobianSO3(pInt->JRg * dbg.cast<float>())
                .cast<double>() *
            pInt->JRg.cast<double>();
        J = SqrtInformation * J;
      }
    }

    return true;
  }

 private:
  ORB_SLAM3::IMU::Preintegrated* pInt;

  const Eigen::Matrix3d Ri, Rj;

  Eigen::Matrix3d SqrtInformation;
};

#endif  // IMU_CERES_H

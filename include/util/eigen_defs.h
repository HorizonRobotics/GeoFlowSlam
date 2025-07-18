
#ifndef UTIL_EIGEN_DEFS_H_
#define UTIL_EIGEN_DEFS_H_

#include <Eigen/Core>

namespace Eigen {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;

}  // namespace Eigen

#endif  // UTIL_EIGEN_DEFS_H_
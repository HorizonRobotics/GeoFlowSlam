
#ifndef UTIL_SVD_H_
#define UTIL_SVD_H_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <limits>

// https://eigen.tuxfamily.org/bz/show_bug.cgi?id=257#c24
template <typename MatType>
Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime,
              MatType::RowsAtCompileTime>
pseudoInverse(const MatType &A,
              typename MatType::Scalar epsilon =
                  std::numeric_limits<typename MatType::Scalar>::epsilon()) {
  using WorkingMatType =
      Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                    MatType::MaxRowsAtCompileTime,
                    MatType::MaxColsAtCompileTime>;
  Eigen::BDCSVD<WorkingMatType> svd(A,
                                    Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(epsilon * std::max(A.cols(), A.rows()));
  Eigen::Index rank = svd.rank();
  Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic,
                MatType::RowsAtCompileTime, 0,
                Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime,
                MatType::MaxRowsAtCompileTime>
      tmp = svd.matrixU().leftCols(rank).adjoint();
  tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
  return svd.matrixV().leftCols(rank) * tmp;
}

template <typename MatType>
Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime,
              MatType::RowsAtCompileTime>
selfAdjointInverse(const MatType &A) {
  using WorkingMatType =
      Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                    MatType::MaxRowsAtCompileTime,
                    MatType::MaxColsAtCompileTime>;
  Eigen::SelfAdjointEigenSolver<WorkingMatType> svd(A,
                                                    Eigen::ComputeEigenvectors);
  return svd.eigenvectors() * svd.eigenvalues().cwiseInverse().asDiagonal() *
         svd.eigenvectors().adjoint();
}

#endif  // UTIL_SVD_H_

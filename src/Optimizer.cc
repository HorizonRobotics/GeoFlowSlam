/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
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

#include "Optimizer.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <complex>
#include <mutex>
#include <unsupported/Eigen/MatrixFunctions>

#include "Converter.h"
#include "G2oTypes.h"
#include "OptimizableTypes.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace ORB_SLAM3 {
bool sortByVal(const pair<MapPoint*, int>& a, const pair<MapPoint*, int>& b) {
  return (a.second < b.second);
}

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations,
                                       bool* pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
  vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame*>& vpKFs,
                                 const vector<MapPoint*>& vpMP, int nIterations,
                                 bool* pbStopFlag, const unsigned long nLoopKF,
                                 const bool bRobust) {
  vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  Map* pMap = vpKFs[0]->GetMap();

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;

  const int nExpectedSize = (vpKFs.size()) * vpMP.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  // Set KeyFrame vertices

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKF->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKF->mnId);
    vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
  }

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (map<KeyFrame*, tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      KeyFrame* pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid) continue;
      if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
        continue;
      nEdges++;

      const int leftIndex = get<0>(mit->second);

      if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0) {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();
        if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
          continue;
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMP);
      } else if (leftIndex != -1 &&
                 pKF->mvuRight[leftIndex] >= 0)  // Stereo observation
      {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

        Eigen::Matrix<double, 3, 1> obs;
        const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMP);
      }

      if (pKF->mpCamera2) {
        int rightIndex = get<1>(mit->second);

        if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()) {
          rightIndex -= pKF->NLeft;

          Eigen::Matrix<double, 2, 1> obs;
          cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
          obs << kp.pt.x, kp.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZToBody* e =
              new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);

          Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
          e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                 Trl.translation().cast<double>());

          e->pCamera = pKF->mpCamera2;

          optimizer.addEdge(e);
          vpEdgesBody.push_back(e);
          vpEdgeKFBody.push_back(pKF);
          vpMapPointEdgeBody.push_back(pMP);
        }
      }
    }

    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);
  Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

  // Recover optimized data
  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(),
                                SE3quat.translation().cast<float>()));
    } else {
      pKF->mTcwGBA =
          Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      pKF->mnBAGlobalForKF = nLoopKF;

      Sophus::SE3f mTwc = pKF->GetPoseInverse();
      Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
      Eigen::Vector3f vector_dist = mTcGBA_c.translation();
      double dist = vector_dist.norm();
      if (dist > 1) {
        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

        for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++) {
          ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
          MapPoint* pMP = vpMapPointEdgeMono[i2];
          KeyFrame* pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad()) continue;

          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            numMonoBadPoints++;

          } else {
            numMonoOptPoints++;
            vpMonoMPsOpt.push_back(pMP);
          }
        }

        for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++) {
          g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
          MapPoint* pMP = vpMapPointEdgeStereo[i2];
          KeyFrame* pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad()) continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            numStereoBadPoints++;
          } else {
            numStereoOptPoints++;
            vpStereoMPsOpt.push_back(pMP);
          }
        }
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    MapPoint* pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

void Optimizer::FullInertialBA(Map* pMap, int its, const bool bFixLocal,
                               const long unsigned int nLoopId,
                               bool* pbStopFlag, bool bInit, float priorG,
                               float priorA, Eigen::VectorXd* vSingVal,
                               bool* bHess) {
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e-5);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  int nNonFixed = 0;

  // Set KeyFrame vertices
  KeyFrame* pIncKF;
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    pIncKF = pKFi;
    bool bFixed = false;
    if (bFixLocal) {
      bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
               (pKFi->mnBAFixedForKF >= (maxKFid - 1));
      if (!bFixed) nNonFixed++;
      VP->setFixed(bFixed);
    }
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(bFixed);
      optimizer.addVertex(VV);
      if (!bInit) {
        VertexGyroBias* VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pKFi);
        VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
      }
    }
  }

  if (bInit) {
    VertexGyroBias* VG = new VertexGyroBias(pIncKF);
    VG->setId(4 * maxKFid + 2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pIncKF);
    VA->setId(4 * maxKFid + 3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
  }

  if (bFixLocal) {
    if (nNonFixed < 3) return;
  }

  // IMU links
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];

    if (!pKFi->mPrevKF) {
      Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (pKFi->bImu && pKFi->mPrevKF->bImu) {
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
        g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
        g2o::HyperGraph::Vertex* VV1 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

        g2o::HyperGraph::Vertex* VG1;
        g2o::HyperGraph::Vertex* VA1;
        g2o::HyperGraph::Vertex* VG2;
        g2o::HyperGraph::Vertex* VA2;
        if (!bInit) {
          VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
          VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
          VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
          VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
        } else {
          VG1 = optimizer.vertex(4 * maxKFid + 2);
          VA1 = optimizer.vertex(4 * maxKFid + 3);
        }

        g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
        g2o::HyperGraph::Vertex* VV2 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

        if (!bInit) {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                 << endl;
            continue;
          }
        } else {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << endl;
            continue;
          }
        }

        EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
        ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
        ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
        ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
        ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        ei->setRobustKernel(rki);
        rki->setDelta(sqrt(16.0));

        optimizer.addEdge(ei);

        if (!bInit) {
          EdgeGyroRW* egr = new EdgeGyroRW();
          egr->setVertex(0, VG1);
          egr->setVertex(1, VG2);
          Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                      .cast<double>()
                                      .inverse();
          egr->setInformation(InfoG);
          egr->computeError();
          optimizer.addEdge(egr);

          EdgeAccRW* ear = new EdgeAccRW();
          ear->setVertex(0, VA1);
          ear->setVertex(1, VA2);
          Eigen::Matrix3d InfoA =
              pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                  .cast<double>()
                  .inverse();
          ear->setInformation(InfoA);
          ear->computeError();
          optimizer.addEdge(ear);
        }
      } else
        cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu"
             << endl;
    }
  }

  if (bInit) {
    g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
    g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);

    // Add prior to comon biases
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;  //
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);

    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;  //
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);
  }

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  const unsigned long iniMPid = maxKFid * 5;

  vector<bool> vbNotIncludedMP(vpMPs.size(), false);

  for (size_t i = 0; i < vpMPs.size(); i++) {
    MapPoint* pMP = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    bool bAllFixed = true;

    // Set edges
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnId > maxKFid) continue;

      if (!pKFi->isBad()) {
        const int leftIndex = get<0>(mit->second);
        cv::KeyPoint kpUn;

        if (leftIndex != -1 &&
            pKFi->mvuRight[get<0>(mit->second)] < 0)  // Monocular observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
        } else if (leftIndex != -1 &&
                   pKFi->mvuRight[leftIndex] >= 0)  // stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
        }

        if (pKFi->mpCamera2) {  // Monocular right observation
          int rightIndex = get<1>(mit->second);

          if (rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            kpUn = pKFi->mvKeysRight[rightIndex];
            obs << kpUn.pt.x, kpUn.pt.y;

            EdgeMono* e = new EdgeMono(1);

            g2o::OptimizableGraph::Vertex* VP =
                dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                    optimizer.vertex(pKFi->mnId));
            if (bAllFixed)
              if (!VP->fixed()) bAllFixed = false;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, VP);
            e->setMeasurement(obs);
            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
          }
        }
      }
    }

    if (bAllFixed) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    if (nLoopId == 0) {
      Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                       VP->estimate().tcw[0].cast<float>());
      pKFi->SetPose(Tcw);
    } else {
      pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                   VP->estimate().tcw[0].cast<float>());
      pKFi->mnBAGlobalForKF = nLoopId;
    }
    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      if (nLoopId == 0) {
        pKFi->SetVelocity(VV->estimate().cast<float>());
      } else {
        pKFi->mVwbGBA = VV->estimate().cast<float>();
      }

      VertexGyroBias* VG;
      VertexAccBias* VA;
      if (!bInit) {
        VG = static_cast<VertexGyroBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
        VA = static_cast<VertexAccBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      } else {
        VG = static_cast<VertexGyroBias*>(optimizer.vertex(4 * maxKFid + 2));
        VA = static_cast<VertexAccBias*>(optimizer.vertex(4 * maxKFid + 3));
      }

      Vector6d vb;
      vb << VG->estimate(), VA->estimate();
      IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
      if (nLoopId == 0) {
        pKFi->SetNewBias(b);
      } else {
        pKFi->mBiasGBA = b;
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMPs.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    MapPoint* pMP = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));

    if (nLoopId == 0) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopId;
    }
  }

  pMap->IncreaseChangeIndex();
}

int Optimizer::PoseOptimization(Frame* pFrame,
                                const bool bFrame2FrameReprojError,
                                const bool bFrame2MapReprojError,
                                const int nIterations) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float> Tcw = pFrame->GetPose();
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                 Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;
  float reprojectionError = 100.0f;

  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
  vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815);
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Conventional SLAM
        if (!pFrame->mpCamera2) {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0) {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else  // Stereo observation
          {
            // nInitialCorrespondences++;
            // 已经预先标记了outlier
            // 没有通过F Check的点，不参与优化
            // if (pFrame->mvbOutlier[i]) continue;
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            const float& kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
        // SLAM with respect a rigid body
        else {
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;

          if (i < pFrame->Nleft) {  // Left camera observation
            kpUn = pFrame->mvKeys[i];
            if (pFrame->mvbOutlier[i]) continue;
            // pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            if (pFrame->mvbOutlier[i]) continue;
            // pFrame->mvbOutlier[i] = false;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera2;
            e->Xw = pMP->GetWorldPos().cast<double>();

            e->mTrl = g2o::SE3Quat(
                pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
                pFrame->GetRelativePoseTrl().translation().cast<double>());

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
  }

  if (nInitialCorrespondences < 3) return 0;

  // We perform 4 optimizations, after each optimization we classify
  // observation
  // as inlier/outlier At the next optimization, outliers are not included,
  // but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  // const float chi2Stereo[4] = {7.815, 6.815, 5.815, 5.0};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nGood = 0;

  for (size_t it = 0; it < 4; it++) {
    Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      reprojectionError = chi2;

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nGood++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

      const size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      // 验后重投影误差
      const float chi2 = e->chi2();
      reprojectionError = chi2;
      // avgReprojectionError += chi2;

      if (chi2 > chi2Stereo[it]) {
        // 最后一次标记为outlier
        pFrame->mvbOutlier[idx] = true;
        // if (it == 3) pFrame->mvbOutlier[idx] = true;
        // 缩小信息矩阵
        // float scale = chi2 / chi2Stereo[it] * 2.5;
        // Eigen::Matrix3d InfoMatrix = e->information() * scale;
        // e->setInformation(InfoMatrix);
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        nGood++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    avgReprojectionError /= nGood;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    if (optimizer.edges().size() < 10) break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                          SE3quat_recov.translation().cast<float>());
  // cout << "nInitialCorrespondences: " << nInitialCorrespondences << endl;
  // cout << "nBad: " << nBad << endl;
  // cout << "inliers ratio: "
  //      << static_cast<double>(nInitialCorrespondences - nBad) /
  //             nInitialCorrespondences
  //      << endl;
  // cout << "reprojectionError: " << reprojectionError << endl;
  // if (static_cast<double>(nInitialCorrespondences - nBad) /
  //             nInitialCorrespondences >
  //         0.6 &&
  //     reprojectionError < 5.0 && nGood > 10) {
  //   pFrame->SetPose(pose);
  // }
  // pFrame->SetPose(pose);
  return nInitialCorrespondences - nBad;
}


void Optimizer::LocalVisualLidarBA(
    KeyFrame* pKF, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    bool* pbStopFlag, bool pbICPFlag, Map* pMap, int& num_fixedKF,
    int& num_OptKF, int& num_MPs, int& num_edges) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  list<KeyFrame*> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  Map* pCurrentMap = pKF->GetMap();
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
    KeyFrame* pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
      lLocalKeyFrames.push_back(pKFi);
  }

  // Local MapPoints seen in Local KeyFrames
  num_fixedKF = 0;
  list<MapPoint*> lLocalMapPoints;
  set<MapPoint*> sNumObsMP;
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    if (pKFi->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;
    }
    vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
        }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  list<KeyFrame*> lFixedCameras;
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    map<KeyFrame*, tuple<int, int>> observations = (*lit)->GetObservations();
    for (map<KeyFrame*, tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
          lFixedCameras.push_back(pKFi);
      }
    }
  }
  num_fixedKF = lFixedCameras.size() + num_fixedKF;

  if (num_fixedKF == 0) {
    Verbose::PrintMess(
        "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted",
        Verbose::VERBOSITY_NORMAL);
    return;
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // if (pMap->IsInertial())
  //     solver->setUserLambdaInit(100.0);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // DEBUG LBA
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // Set Local KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msOptKFs.insert(pKFi->mnId);
  }
  num_OptKF = lLocalKeyFrames.size();

  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(),
                                 lend = lFixedCameras.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msFixedKFs.insert(pKFi->mnId);
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);
  const float thHuberICP = sqrt(4.0);

  int nPoints = 0;

  int nEdges = 0;
  vector<g2o::EdgeSim3*> veICP;
  veICP.reserve(lLocalKeyFrames.size());
  if (pbICPFlag) {
    size_t index = 0;
    for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                   lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
      KeyFrame* pKFi = *lit;
      if (pKFi->mnMatchesInliers > 75) continue;
      Sophus::SE3d Tciw = pKFi->GetPose().cast<double>();
      g2o::Sim3 Sciw(Tciw.unit_quaternion(), Tciw.translation(), 1.0);
      if (pKFi->mPrevKF) {
        Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
        Sophus::SE3d Tcjw = pKFi->mPrevKF->GetPose().cast<double>();
        g2o::Sim3 Scjw(Tcjw.unit_quaternion(), Tcjw.translation(), 1.0);
        g2o::Sim3 Scjci = Scjw * Sciw.inverse();
        Eigen::Matrix4d Tcjci;
        Tcjci.block<3, 3>(0, 0) = Scjci.rotation().toRotationMatrix();
        Tcjci.block<3, 1>(0, 3) = Scjci.translation();
        Tcjci(3, 3) = 1.;
        init_T.matrix() = Tcjci;

        g2o::VertexSim3Expmap* vPrevKF = dynamic_cast<g2o::VertexSim3Expmap*>(
            optimizer.vertex(pKFi->mPrevKF->mnId));
        g2o::VertexSim3Expmap* vCurrKF =
            dynamic_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mnId));
        if (!vPrevKF || !vCurrKF) continue;
        RegistrationResult result = mpRegistration->RegisterPointClouds(
            *(pKFi->mPrevKF->source_points), *(pKFi->source_points), init_T);
        Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
        g2o::Sim3 icp_sim3 =
            g2o::Sim3(relative_pose_icp.block<3, 3>(0, 0),
                      relative_pose_icp.block<3, 1>(0, 3), 1.0);
        if (result.converged && result.num_inliers > 100 &&
            (result.error / result.num_inliers) < 0.01) {
          g2o::EdgeSim3* eICP = new g2o::EdgeSim3();
          eICP->setVertex(0, vPrevKF);
          eICP->setVertex(1, vCurrKF);
          // eICP->setInformation(Eigen::Matrix<double, 7, 7>::Identity() *
          // 1e2);
          eICP->setMeasurement(icp_sim3);
          eICP->computeError();
          // std::cout << "icp relative pose error:" << eICP->error() << endl;
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          eICP->setRobustKernel(rk);
          rk->setDelta(thHuberICP);
          optimizer.addEdge(eICP);
          veICP.push_back(eICP);
        }
      }
      index++;
    }
  }

  // Lidar edges
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    if (pKFi->mnMatchesInliers > 75) continue;
    vector<EdgeSE3LidarPoint2Plane*> vpEdgesLidarPoint2Plane;
    Eigen::Matrix4d initPose = Converter::toMatrix4d(pKFi->GetPose().inverse());
    vpEdgesLidarPoint2Plane =
        GenerateLidarEdge<EdgeSE3LidarPoint2Plane, KeyFrame>(
            pKFi, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, static_cast<g2o::VertexSim3Expmap*>(
                               optimizer.vertex(pKFi->mnId)));
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;
      }
    }
  }
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    nPoints++;

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Set edges
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(mit->second);

        // Monocular observation
        if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();
          if (optimizer.vertex(id) == NULL ||
              optimizer.vertex(pKFi->mnId) == NULL)
            continue;
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          e->pCamera = pKFi->mpCamera;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);

          nEdges++;
        } else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >=
                                          0)  // Stereo observation
        {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
          if (optimizer.vertex(id) == NULL ||
              optimizer.vertex(pKFi->mnId) == NULL)
            continue;
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          e->fx = pKFi->fx;
          e->fy = pKFi->fy;
          e->cx = pKFi->cx;
          e->cy = pKFi->cy;
          e->bf = pKFi->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);

          nEdges++;
        }

        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(mit->second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZToBody();
            if (optimizer.vertex(id) == NULL ||
                optimizer.vertex(pKFi->mnId) == NULL)
              continue;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                   Trl.translation().cast<double>());

            e->pCamera = pKFi->mpCamera2;

            optimizer.addEdge(e);
            vpEdgesBody.push_back(e);
            vpEdgeKFBody.push_back(pKFi);
            vpMapPointEdgeBody.push_back(pMP);

            nEdges++;
          }
        }
      }
    }
  }
  num_edges = nEdges;

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                   vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
    MapPoint* pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFBody[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());
    pKFi->SetPose(Tiw);
  }

  // Points
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}
void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag,
                                      bool pbICPFlag, Map* pMap,
                                      int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  list<KeyFrame*> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  Map* pCurrentMap = pKF->GetMap();
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
    KeyFrame* pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
      lLocalKeyFrames.push_back(pKFi);
  }

  // Local MapPoints seen in Local KeyFrames
  num_fixedKF = 0;
  list<MapPoint*> lLocalMapPoints;
  set<MapPoint*> sNumObsMP;
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    if (pKFi->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;
    }
    vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
        }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  list<KeyFrame*> lFixedCameras;
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    map<KeyFrame*, tuple<int, int>> observations = (*lit)->GetObservations();
    for (map<KeyFrame*, tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
          lFixedCameras.push_back(pKFi);
      }
    }
  }
  num_fixedKF = lFixedCameras.size() + num_fixedKF;

  if (num_fixedKF == 0) {
    Verbose::PrintMess(
        "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted",
        Verbose::VERBOSITY_NORMAL);
    return;
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // if (pMap->IsInertial())
  //     solver->setUserLambdaInit(100.0);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // DEBUG LBA
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // Set Local KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msOptKFs.insert(pKFi->mnId);
  }
  num_OptKF = lLocalKeyFrames.size();

  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(),
                                 lend = lFixedCameras.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msFixedKFs.insert(pKFi->mnId);
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);
  const float thHuberICP = sqrt(4.0);

  int nPoints = 0;

  int nEdges = 0;
  vector<g2o::EdgeSim3*> veICP;
  veICP.reserve(lLocalKeyFrames.size());
  if (pbICPFlag) {
    size_t index = 0;
    for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                   lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
      // if (index % 2 != 0) continue;
      KeyFrame* pKFi = *lit;
      if (pKFi->mnMatchesInliers > 75) continue;
      Sophus::SE3d Tciw = pKFi->GetPose().cast<double>();
      g2o::Sim3 Sciw(Tciw.unit_quaternion(), Tciw.translation(), 1.0);
      if (pKFi->mPrevKF) {
        Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
        // 当前帧相对于上一帧的变换
        Sophus::SE3d Tcjw = pKFi->mPrevKF->GetPose().cast<double>();
        g2o::Sim3 Scjw(Tcjw.unit_quaternion(), Tcjw.translation(), 1.0);
        g2o::Sim3 Scjci = Scjw * Sciw.inverse();
        Eigen::Matrix4d Tcjci;
        Tcjci.block<3, 3>(0, 0) = Scjci.rotation().toRotationMatrix();
        Tcjci.block<3, 1>(0, 3) = Scjci.translation();
        Tcjci(3, 3) = 1.;
        init_T.matrix() = Tcjci;

        g2o::VertexSim3Expmap* vPrevKF = dynamic_cast<g2o::VertexSim3Expmap*>(
            optimizer.vertex(pKFi->mPrevKF->mnId));
        g2o::VertexSim3Expmap* vCurrKF =
            dynamic_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mnId));
        if (!vPrevKF || !vCurrKF) continue;
        RegistrationResult result = mpRegistration->RegisterPointClouds(
            *(pKFi->mPrevKF->source_points), *(pKFi->source_points), init_T);
        Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
        g2o::Sim3 icp_sim3 =
            g2o::Sim3(relative_pose_icp.block<3, 3>(0, 0),
                      relative_pose_icp.block<3, 1>(0, 3), 1.0);
        if (result.converged && result.num_inliers > 100 &&
            (result.error / result.num_inliers) < 0.01) {
          g2o::EdgeSim3* eICP = new g2o::EdgeSim3();
          eICP->setVertex(0, vPrevKF);
          eICP->setVertex(1, vCurrKF);
          // eICP->setInformation(Eigen::Matrix<double, 7, 7>::Identity() *
          // 1e2);
          eICP->setMeasurement(icp_sim3);
          eICP->computeError();
          // std::cout << "icp relative pose error:" << eICP->error() << endl;
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          eICP->setRobustKernel(rk);
          rk->setDelta(thHuberICP);
          optimizer.addEdge(eICP);
          veICP.push_back(eICP);
        }
      }
      index++;
    }
  }

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    nPoints++;

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Set edges
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(mit->second);

        // Monocular observation
        if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();
          if (optimizer.vertex(id) == NULL ||
              optimizer.vertex(pKFi->mnId) == NULL)
            continue;
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          e->pCamera = pKFi->mpCamera;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);

          nEdges++;
        } else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >=
                                          0)  // Stereo observation
        {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
          if (optimizer.vertex(id) == NULL ||
              optimizer.vertex(pKFi->mnId) == NULL)
            continue;
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          e->fx = pKFi->fx;
          e->fy = pKFi->fy;
          e->cx = pKFi->cx;
          e->cy = pKFi->cy;
          e->bf = pKFi->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);

          nEdges++;
        }

        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(mit->second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZToBody();
            if (optimizer.vertex(id) == NULL ||
                optimizer.vertex(pKFi->mnId) == NULL)
              continue;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                   Trl.translation().cast<double>());

            e->pCamera = pKFi->mpCamera2;

            optimizer.addEdge(e);
            vpEdgesBody.push_back(e);
            vpEdgeKFBody.push_back(pKFi);
            vpMapPointEdgeBody.push_back(pMP);

            nEdges++;
          }
        }
      }
    }
  }
  num_edges = nEdges;

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                   vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
    MapPoint* pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFBody[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());
    pKFi->SetPose(Tiw);
  }

  // Points
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<KeyFrame*, set<KeyFrame*>>& LoopConnections,
    const bool& bFixScale, const bool& bUseICPConstraint) {
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1);  // For debugging
  Eigen::Vector3d z_vec;
  z_vec << 0.0, 0.0, 1.0;

  const int minFeat = 100;

  // Set KeyFrame vertices
  cout << "Setting KeyFrame vertices" << endl;
  cout << "KeyFrame sizes " << vpKFs.size() << endl;
  // 创建一个map储存LoopConnections的mnId
  std::set<long unsigned int> connectedKeyFramesID1;
  std::set<long unsigned int> connectedKeyFramesID2;
  int min_id = 1e9;
  int max_id = 0;
  for (const auto& loopConnection : LoopConnections) {
    int keyFrameId = loopConnection.first->mnId;
    connectedKeyFramesID1.insert(keyFrameId);
    std::set<long unsigned int> connectedKeyFrames;
    min_id = std::min(min_id, keyFrameId);
    max_id = std::max(max_id, keyFrameId);
    for (const auto& connectedKeyFrame : loopConnection.second) {
      min_id = std::min(min_id, static_cast<int>(connectedKeyFrame->mnId));
      max_id = std::max(max_id, static_cast<int>(connectedKeyFrame->mnId));
      connectedKeyFramesID2.insert(connectedKeyFrame->mnId);
    }
  }

  cout << "min_id: " << min_id << ", max_id: " << max_id << endl;
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;

    const int nIDi = pKF->mnId;
    // 遍历mapLoopConnections
    // if (nIDi < min_id || nIDi > max_id) continue;
    bool is_loop_frame = false;
    if (connectedKeyFramesID1.find(nIDi) != connectedKeyFramesID1.end() ||
        connectedKeyFramesID2.find(nIDi) != connectedKeyFramesID2.end()) {
      is_loop_frame = true;
    }

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    if (pKF->mnId == pMap->GetInitKFid()) VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = bFixScale;

    optimizer.addVertex(VSim3);
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;  // For debugging

    vpVertices[nIDi] = VSim3;
  }
  cout << "Setting KeyFrame vertices done" << endl;
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // Set Loop edges
  cout << "Setting Loop edges" << endl;
  int count_loop = 0;
  int count_connections = 0;
  for (map<KeyFrame*, set<KeyFrame*>>::const_iterator
           mit = LoopConnections.begin(),
           mend = LoopConnections.end();
       mit != mend; mit++, count_connections++) {
    // 对LoopConnections进行采样，保留一半的边
    KeyFrame* pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const set<KeyFrame*>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];
    const g2o::Sim3 Swi = Siw.inverse();

    for (set<KeyFrame*>::const_iterator sit = spConnections.begin(),
                                        send = spConnections.end();
         sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(*sit) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      if (optimizer.vertex(nIDj) == NULL || optimizer.vertex(nIDi) == NULL)
        continue;
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);
      // You can set different measurements for the edge here if needed
      // e.g., e->setMeasurement(anotherMeasurement);

      e->information() = matLambda;

      optimizer.addEdge(e);
      count_loop++;
      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }
  cout << "Setting Loop edges done" << endl;
  // Set normal edges
  cout << "Setting normal edges" << endl;
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    KeyFrame* pKF = vpKFs[i];

    const int nIDi = pKF->mnId;

    g2o::Sim3 Swi;

    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Swi = (iti->second).inverse();
    else
      Swi = vScw[nIDi].inverse();

    KeyFrame* pParentKF = pKF->GetParent();

    // Spanning tree edge
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];

      g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      if (optimizer.vertex(nIDj) == NULL || optimizer.vertex(nIDi) == NULL)
        continue;
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // Loop edges
    const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
    for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      KeyFrame* pLKF = *sit;
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Slw;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        g2o::Sim3 Sli = Slw * Swi;
        g2o::EdgeSim3* el = new g2o::EdgeSim3();
        if (optimizer.vertex(pLKF->mnId) == NULL ||
            optimizer.vertex(nIDi) == NULL)
          continue;
        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(pLKF->mnId)));
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;
        optimizer.addEdge(el);

        // TODO: Add ICP edges
        if (bUseICPConstraint) {
          Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
          init_T.linear() = Sli.rotation().toRotationMatrix();
          init_T.translation() = Sli.translation() / Sli.scale();

          RegistrationResult result = mpRegistration->RegisterPointClouds(
              *(pLKF->source_points), *(pKF->source_points), init_T);
          Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
          g2o::Sim3 icp_sim3 =
              g2o::Sim3(relative_pose_icp.block<3, 3>(0, 0),
                        relative_pose_icp.block<3, 1>(0, 3), 1.0);
          if (result.converged && result.num_inliers > 100 &&
              (result.error / result.num_inliers) < 0.1) {
            g2o::EdgeSim3* eicp = new g2o::EdgeSim3();
            eicp->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(nIDi)));
            eicp->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pLKF->mnId)));
            eicp->setMeasurement(icp_sim3);
            eicp->information() = matLambda * 3;
            optimizer.addEdge(eicp);
          }
        }
      }
    }

    // Covisibility graph edges
    // cout << "Covisibility graph edges" << endl;
    const vector<KeyFrame*> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      KeyFrame* pKFn = *vit;
      if (pKFn && pKFn != pParentKF &&
          !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Snw;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          g2o::Sim3 Sni = Snw * Swi;

          g2o::EdgeSim3* en = new g2o::EdgeSim3();
          if (optimizer.vertex(pKFn->mnId) == NULL ||
              optimizer.vertex(nIDi) == NULL)
            continue;
          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pKFn->mnId)));
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;
          optimizer.addEdge(en);
        }
      }
    }

    // Inertial edges if inertial
    if (pKF->bImu && pKF->mPrevKF) {
      g2o::Sim3 Spw;
      LoopClosing::KeyFrameAndPose::const_iterator itp =
          NonCorrectedSim3.find(pKF->mPrevKF);
      if (itp != NonCorrectedSim3.end())
        Spw = itp->second;
      else
        Spw = vScw[pKF->mPrevKF->mnId];

      g2o::Sim3 Spi = Spw * Swi;
      g2o::EdgeSim3* ep = new g2o::EdgeSim3();
      if (optimizer.vertex(pKF->mPrevKF->mnId) == NULL ||
          optimizer.vertex(nIDi) == NULL)
        continue;
      ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(pKF->mPrevKF->mnId)));
      ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(nIDi)));
      ep->setMeasurement(Spi);
      ep->information() = matLambda;
      optimizer.addEdge(ep);
    }
  }
  cout << "Setting normal edges done" << endl;
  cout << "OptimizeEssentialGraph" << endl;
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  bool online = false;
  optimizer.optimize(20, online);

  optimizer.computeActiveErrors();
  cout << "OptimizeEssentialGraph done" << endl;
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);
  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  cout << "SE3 Pose Recovering" << endl;
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (!pKFi) continue;
    const int nIDi = pKFi->mnId;
    if (optimizer.vertex(nIDi) == NULL) continue;
    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();

    Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                     CorrectedSiw.translation().cast<float>() / s);
    pKFi->SetPose(Tiw);
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;
    if (pMP->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = pMP->mnCorrectedReference;
    } else {
      KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;
    }

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }

  // TODO Check this changeindex
  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(KeyFrame* pCurKF,
                                       vector<KeyFrame*>& vpFixedKFs,
                                       vector<KeyFrame*>& vpFixedCorrectedKFs,
                                       vector<KeyFrame*>& vpNonFixedKFs,
                                       vector<MapPoint*>& vpNonCorrectedMPs,
                                       const bool& bUseICPConstraint) {
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedKFs.size()) +
                         " KFs fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedCorrectedKFs.size()) +
                         " KFs fixed in the old map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonFixedKFs.size()) +
                         " KFs non-fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonCorrectedMPs.size()) +
                         " MPs non-corrected in the merged map",
                     Verbose::VERBOSITY_DEBUG);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  Map* pMap = pCurKF->GetMap();
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  vector<bool> vpGoodPose(nMaxKFid + 1);
  vector<bool> vpBadPose(nMaxKFid + 1);

  const int minFeat = 100;

  for (KeyFrame* pKFi : vpFixedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = true;

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = false;
  }
  Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded",
                     Verbose::VERBOSITY_DEBUG);

  set<unsigned long> sIdKF;
  for (KeyFrame* pKFi : vpFixedCorrectedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
    vScw[nIDi] =
        g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = true;
  }

  for (KeyFrame* pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    if (sIdKF.count(nIDi))  // It has already added in the corrected merge KFs
      continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    VSim3->setFixed(false);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi] = true;
  }

  vector<KeyFrame*> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
               vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  set<KeyFrame*> spKFs(vpKFs.begin(), vpKFs.end());

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  for (KeyFrame* pKFi : vpKFs) {
    int num_connections = 0;
    const int nIDi = pKFi->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    if (vpGoodPose[nIDi]) correctedSwi = vCorrectedSwc[nIDi];
    if (vpBadPose[nIDi]) Swi = vScw[nIDi].inverse();

    KeyFrame* pParentKFi = pKFi->GetParent();

    // Spanning tree edge
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      int nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool bHasRelation = false;

      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw = vScw[nIDj];
        bHasRelation = true;
      }

      if (bHasRelation) {
        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
        num_connections++;
      }
    }

    // Loop edges
    const set<KeyFrame*> sLoopEdges = pKFi->GetLoopEdges();
    for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      KeyFrame* pLKF = *sit;
      if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
        g2o::Sim3 Slw;
        bool bHasRelation = false;

        if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
          Slw = vCorrectedSwc[pLKF->mnId].inverse();
          bHasRelation = true;
        } else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
          Slw = vScw[pLKF->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3* el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pLKF->mnId)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          num_connections++;
          // TODO: Add ICP edges
          if (bUseICPConstraint) {
            Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
            // 回环帧到当前帧的变换
            init_T.linear() = Sli.rotation().toRotationMatrix();
            init_T.translation() = Sli.translation() / Sli.scale();

            RegistrationResult result = mpRegistration->RegisterPointClouds(
                *(pLKF->source_points), *(pKFi->source_points), init_T);
            Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
            g2o::Sim3 icp_sim3 =
                g2o::Sim3(relative_pose_icp.block<3, 3>(0, 0),
                          relative_pose_icp.block<3, 1>(0, 3), 1.0);
            if (result.converged && result.num_inliers > 100 &&
                (result.error / result.num_inliers) < 0.2) {
              g2o::EdgeSim3* eicp = new g2o::EdgeSim3();
              eicp->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(nIDi)));
              eicp->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pLKF->mnId)));
              eicp->setMeasurement(icp_sim3);
              eicp->information() = matLambda;
              optimizer.addEdge(eicp);
            }
          }
        }
      }
    }

    // Covisibility graph edges
    const vector<KeyFrame*> vpConnectedKFs =
        pKFi->GetCovisiblesByWeight(minFeat);
    for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      KeyFrame* pKFn = *vit;
      if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
          !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
        if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
          g2o::Sim3 Snw = vScw[pKFn->mnId];
          bool bHasRelation = false;

          if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
            Snw = vCorrectedSwc[pKFn->mnId].inverse();
            bHasRelation = true;
          } else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
            Snw = vScw[pKFn->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(pKFn->mnId)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            num_connections++;

            // TODO: Add ICP edges
            if (bUseICPConstraint) {
              Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
              // 回环帧到当前帧的变换
              init_T.linear() = Sni.rotation().toRotationMatrix();
              init_T.translation() = Sni.translation() / Sni.scale();

              RegistrationResult result = mpRegistration->RegisterPointClouds(
                  *(pKFn->source_points), *(pKFi->source_points), init_T);
              Eigen::Matrix4d relative_pose_icp =
                  result.T_target_source.matrix();
              g2o::Sim3 icp_sim3 =
                  g2o::Sim3(relative_pose_icp.block<3, 3>(0, 0),
                            relative_pose_icp.block<3, 1>(0, 3), 1.0);
              if (result.converged && result.num_inliers > 100 &&
                  (result.error / result.num_inliers) < 0.2) {
                g2o::EdgeSim3* eicp = new g2o::EdgeSim3();
                eicp->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                       optimizer.vertex(nIDi)));
                eicp->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                       optimizer.vertex(pKFn->mnId)));
                eicp->setMeasurement(icp_sim3);
                eicp->information() = matLambda;
                optimizer.addEdge(eicp);
              }
            }
          }
        }
      }
    }

    if (num_connections == 0) {
      Verbose::PrintMess(
          "Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
          Verbose::VERBOSITY_DEBUG);
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (KeyFrame* pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    pKFi->mTcwBefMerge = pKFi->GetPose();
    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (MapPoint* pMPi : vpNonCorrectedMPs) {
    if (pMPi->isBad()) continue;

    KeyFrame* pRefKF = pMPi->GetReferenceKeyFrame();
    while (pRefKF->isBad()) {
      if (!pRefKF) {
        Verbose::PrintMess(
            "MP " + to_string(pMPi->mnId) + " without a valid reference KF",
            Verbose::VERBOSITY_DEBUG);
        break;
      }

      pMPi->EraseObservation(pRefKF);
      pRefKF = pMPi->GetReferenceKeyFrame();
    }

    if (vpBadPose[pRefKF->mnId]) {
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      Sophus::SE3f Twr = pRefKF->GetPoseInverse();

      Eigen::Vector3f eigCorrectedP3Dw =
          Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
      pMPi->SetWorldPos(eigCorrectedP3Dw);

      pMPi->UpdateNormalAndDepth();
    } else {
      cout << "ERROR: MapPoint has a reference KF from another map" << endl;
    }
  }
}

int Optimizer::OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2,
                            vector<MapPoint*>& vpMatches1, g2o::Sim3& g2oS12,
                            const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian,
                            const bool bAllPoints) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Camera poses
  const Eigen::Matrix3f R1w = pKF1->GetRotation();
  const Eigen::Vector3f t1w = pKF1->GetTranslation();
  const Eigen::Matrix3f R2w = pKF2->GetRotation();
  const Eigen::Vector3f t2w = pKF2->GetTranslation();

  // Set Sim3 vertex
  ORB_SLAM3::VertexSim3Expmap* vSim3 = new ORB_SLAM3::VertexSim3Expmap();
  vSim3->_fix_scale = bFixScale;
  vSim3->setEstimate(g2oS12);
  vSim3->setId(0);
  vSim3->setFixed(false);
  vSim3->pCamera1 = pKF1->mpCamera;
  vSim3->pCamera2 = pKF2->mpCamera;
  optimizer.addVertex(vSim3);

  // Set MapPoint vertices
  const int N = vpMatches1.size();
  const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
  vector<ORB_SLAM3::EdgeSim3ProjectXYZ*> vpEdges12;
  vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*> vpEdges21;
  vector<size_t> vnIndexEdge;
  vector<bool> vbIsInKF2;

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);
  vbIsInKF2.reserve(2 * N);

  const float deltaHuber = sqrt(th2);

  int nCorrespondences = 0;
  int nBadMPs = 0;
  int nInKF2 = 0;
  int nOutKF2 = 0;
  int nMatchWithoutMP = 0;

  vector<int> vIdsOnlyInKF2;

  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i]) continue;

    MapPoint* pMP1 = vpMapPoints1[i];
    MapPoint* pMP2 = vpMatches1[i];

    const int id1 = 2 * i + 1;
    const int id2 = 2 * (i + 1);

    const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

    Eigen::Vector3f P3D1c;
    Eigen::Vector3f P3D2c;

    if (pMP1 && pMP2) {
      if (!pMP1->isBad() && !pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
        P3D1c = R1w * P3D1w + t1w;
        vPoint1->setEstimate(P3D1c.cast<double>());
        vPoint1->setId(id1);
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);

        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else {
        nBadMPs++;
        continue;
      }
    } else {
      nMatchWithoutMP++;

      // TODO The 3D position in KF1 doesn't exist
      if (!pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);

        vIdsOnlyInKF2.push_back(id2);
      }
      continue;
    }

    if (i2 < 0 && !bAllPoints) {
      Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) +
                             "; bAllPoints: " + to_string(bAllPoints),
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    if (P3D2c(2) < 0) {
      Verbose::PrintMess("Sim3: Z coordinate is negative",
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    nCorrespondences++;

    // Set edge x1 = S12*X2
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

    e12->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
    e12->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e12->setMeasurement(obs1);
    const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);
    optimizer.addEdge(e12);

    // Set edge x2 = S21*X1
    Eigen::Matrix<double, 2, 1> obs2;
    cv::KeyPoint kpUn2;
    bool inKF2;
    if (i2 >= 0) {
      kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;
      inKF2 = true;

      nInKF2++;
    } else {
      float invz = 1 / P3D2c(2);
      float x = P3D2c(0) * invz;
      float y = P3D2c(1) * invz;

      obs2 << x, y;
      kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

      inKF2 = false;
      nOutKF2++;
    }

    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 =
        new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

    e21->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
    e21->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

    g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    optimizer.addEdge(e21);

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);

    vbIsInKF2.push_back(inKF2);
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inliers
  int nBad = 0;
  int nBadOutKF2 = 0;
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);
      optimizer.removeEdge(e12);
      optimizer.removeEdge(e21);
      vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ*>(NULL);
      vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*>(NULL);
      nBad++;

      if (!vbIsInKF2[i]) {
        nBadOutKF2++;
      }
      continue;
    }

    // Check if remove the robust adjustment improve the result
    e12->setRobustKernel(0);
    e21->setRobustKernel(0);
  }

  int nMoreIterations;
  if (nBad > 0)
    nMoreIterations = 10;
  else
    nMoreIterations = 5;

  if (nCorrespondences - nBad < 10) return 0;

  // Optimize again only with inliers
  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  int nIn = 0;
  mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    e12->computeError();
    e21->computeError();

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);
    } else {
      nIn++;
    }
  }

  // Recover optimized Sim3
  g2o::VertexSim3Expmap* vSim3_recov =
      static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
  g2oS12 = vSim3_recov->estimate();

  return nIn;
}

void Optimizer::LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, bool pbICPFlag,
                                Map* pMap, int& num_fixedKF, int& num_OptKF,
                                int& num_MPs, int& num_edges, bool bLarge,
                                bool bRecInit) {
  Map* pCurrentMap = pKF->GetMap();

  int maxOpt = 10;
  int opt_it = 8;
  if (bLarge) {
    maxOpt = 20;
    opt_it = 4;
  }
  const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  vector<KeyFrame*> vpOptimizableKFs;
  const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  list<KeyFrame*> lpOptVisKFs;
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else
      break;
  }

  int N = vpOptimizableKFs.size();

  // Optimizable points seen by temporal optimizable keyframes
  list<MapPoint*> lLocalMapPoints;
  for (int i = 0; i < N; i++) {
    vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
    }
  }

  // Fixed Keyframe: First frame previous KF to optimization window)
  list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Optimizable visual KFs
  const int maxCovKF = 0;
  for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
    if (lpOptVisKFs.size() >= maxCovKF) break;

    KeyFrame* pKFi = vpNeighsKFs[i];
    if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
      continue;
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
      lpOptVisKFs.push_back(pKFi);

      vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
      for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
           vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP)
          if (!pMP->isBad())
            if (pMP->mnBALocalForKF != pKF->mnId) {
              lLocalMapPoints.push_back(pMP);
              pMP->mnBALocalForKF = pKF->mnId;
            }
      }
    }
  }

  // Fixed KFs which are not covisible optimizable
  const int maxFixKF = 200;

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    map<KeyFrame*, tuple<int, int>> observations = (*lit)->GetObservations();
    for (map<KeyFrame*, tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) {
          lFixedKeyFrames.push_back(pKFi);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) break;
  }

  bool bNonFixed = (lFixedKeyFrames.size() == 0);

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  if (bLarge) {
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(
        1e-2);  // to avoid iterating for finding optimal lambda
    optimizer.setAlgorithm(solver);
  } else {
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e0);
    optimizer.setAlgorithm(solver);
  }

  // Set Local temporal KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local visual KeyFrame vertices
  for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                 itEnd = lpOptVisKFs.end();
       it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu)  // This should be done only for keyframe just before
                     // temporal window
    {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // TODO: Add ICP edges
  // Create ICP constraints
  vector<EdgeICP*> veICP;
  const float thHuberICP = sqrt(0.4);
  veICP.reserve(N);
  if (pbICPFlag) {
    size_t icp_count = 0;
    // 只约束当前帧和上一帧
    for (int i = 0; i < N; i++) {
      KeyFrame* pKFi = vpOptimizableKFs[i];
      // 降采样
      if (pKFi->mnMatchesInliers > 75) continue;
      Sophus::SE3d Tciw = pKFi->GetPose().cast<double>();
      g2o::Sim3 Sciw(Tciw.unit_quaternion(), Tciw.translation(), 1.0);
      Sophus::SE3d Tcjw = pKFi->mPrevKF->GetPose().cast<double>();
      g2o::Sim3 Scjw(Tcjw.unit_quaternion(), Tcjw.translation(), 1.0);
      g2o::Sim3 Scjci = Scjw * Sciw.inverse();
      Eigen::Matrix4d Tcjci;
      Tcjci.block<3, 3>(0, 0) = Scjci.rotation().toRotationMatrix();
      Tcjci.block<3, 1>(0, 3) = Scjci.translation();
      Tcjci(3, 3) = 1.;
      // cout << ": " << Tcjci << endl;
      float delat_dist =
          sqrt(Scjci.translation().x() * Scjci.translation().x() +
               Scjci.translation().y() * Scjci.translation().y());
      float delat_time = pKFi->mTimeStamp - pKFi->mPrevKF->mTimeStamp;
      // float similarity = mpVoc->score(pKFi->mPrevKF->mBowVec, pKFi->mBowVec);
      if (pKFi->mPrevKF) {
        icp_count++;
        Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
        init_T.matrix() = Tcjci;
        VertexPose* vPrevKF =
            static_cast<VertexPose*>(optimizer.vertex(pKFi->mPrevKF->mnId));
        VertexPose* vKF =
            static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if (!vPrevKF || !vKF) continue;
        RegistrationResult result = mpRegistration->RegisterPointClouds(
            *(pKFi->mPrevKF->source_points), *(pKFi->source_points), init_T);
        Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
        Eigen::Matrix4d delta_pose = relative_pose_icp * Tcjci.inverse();
        float delta_dist = sqrt(delta_pose.block<3, 1>(0, 3).x() *
                                    delta_pose.block<3, 1>(0, 3).x() +
                                delta_pose.block<3, 1>(0, 3).y() *
                                    delta_pose.block<3, 1>(0, 3).y());
        if (result.converged && result.num_inliers > 400 &&
            (result.error / result.num_inliers) < 0.01 && delta_dist < 0.1) {
          // cout << "ICP Pose In LocalInertialBA: " << endl;
          EdgeICP* eICP = new EdgeICP(relative_pose_icp.block<3, 3>(0, 0),
                                      relative_pose_icp.block<3, 1>(0, 3));
          eICP->setVertex(0, vPrevKF);
          eICP->setVertex(1, vKF);
          Eigen::Matrix<double, 6, 6> Info =
              Eigen::Matrix<double, 6, 6>::Identity();
          Info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e2;
          Info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e2;
          eICP->setInformation(Info);
          eICP->computeError();
          // std::cout << "icp relative pose error:" << eICP->error() << endl;
          optimizer.addEdge(eICP);
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          eICP->setRobustKernel(rk);
          rk->setDelta(thHuberICP);
          veICP.push_back(eICP);
        }
      }
    }
  }

  // Create intertial constraints
  vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
  vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)NULL);
  vector<EdgeAccRW*> vear(N, (EdgeAccRW*)NULL);

  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // if (i == N - 1 || bRecInit) {
      // All inertial residuals are included without robust cost function,
      // but not that one linking the last optimizable keyframe inside of
      // the local window and the first fixed keyframe out. The information
      // matrix for this measurement is also downweighted. This is done to
      // avoid accumulating error due to fixing variables.
      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
      rki->setDelta(sqrt(16.0));
      // }
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);

      optimizer.addEdge(vear[i]);
    } else
      cout << "ERROR building inertial edge" << endl;
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;
  const float chi2ICP2 = 0.04;

  const unsigned long iniMPid = maxKFid * 5;

  map<int, int> mVisEdges;
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    mVisEdges[pKFi->mnId] = 0;
  }
  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    mVisEdges[(*lit)->mnId] = 0;
  }

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)
        continue;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(mit->second);

        cv::KeyPoint kpUn;

        // Monocular left observation
        if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
          mVisEdges[pKFi->mnId]++;

          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs);

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // Stereo-observation
        else if (leftIndex != -1)  // Stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          mVisEdges[pKFi->mnId]++;

          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }

        // Monocular right observation
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(mit->second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;
            mVisEdges[pKFi->mnId]++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            EdgeMono* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            // Add here uncerteinty
            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
        }
      }
    }
  }

  // cout << "Total map points: " << lLocalMapPoints.size() << endl;
  for (map<int, int>::iterator mit = mVisEdges.begin(), mend = mVisEdges.end();
       mit != mend; mit++) {
    assert(mit->second >= 3);
  }

  LOG(INFO) << "LocalInertialBA: KFs: " << N
            << ", MPs: " << lLocalMapPoints.size()
            << ", Edges: " << vpEdgesMono.size() + vpEdgesStereo.size();
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(opt_it);  // Originally to 2
  float err_end = optimizer.activeRobustChi2();
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    bool bClose = pMP->mTrackDepth < 10.f;

    if (pMP->isBad()) continue;

    if ((e->chi2() > chi2Mono2 && !bClose) ||
        (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // TODO: Some convergence problems have been detected here
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge)  // bGN)
  {
    cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
    return;
  }

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++)
    (*lit)->mnBAFixedForKF = 0;

  // Recover optimized data
  // Local temporal Keyframes
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Local visual KeyFrame
  for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                 itEnd = lpOptVisKFs.end();
       it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;
  }

  // Points
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

void Optimizer::LocalVisualLidarInertialBA(
    KeyFrame* pKF, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    bool* pbStopFlag, bool pbICPFlag, Map* pMap, int& num_fixedKF,
    int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit) {
  Map* pCurrentMap = pKF->GetMap();

  int maxOpt = 10;
  int opt_it = 8;
  if (bLarge) {
    maxOpt = 20;
    opt_it = 4;
  }
  const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  vector<KeyFrame*> vpOptimizableKFs;
  const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  list<KeyFrame*> lpOptVisKFs;
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else
      break;
  }

  int N = vpOptimizableKFs.size();

  // Optimizable points seen by temporal optimizable keyframes
  list<MapPoint*> lLocalMapPoints;
  for (int i = 0; i < N; i++) {
    vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
    }
  }

  // Fixed Keyframe: First frame previous KF to optimization window)
  list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Optimizable visual KFs
  const int maxCovKF = 0;
  for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
    if (lpOptVisKFs.size() >= maxCovKF) break;

    KeyFrame* pKFi = vpNeighsKFs[i];
    if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
      continue;
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
      lpOptVisKFs.push_back(pKFi);

      vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
      for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
           vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP)
          if (!pMP->isBad())
            if (pMP->mnBALocalForKF != pKF->mnId) {
              lLocalMapPoints.push_back(pMP);
              pMP->mnBALocalForKF = pKF->mnId;
            }
      }
    }
  }

  // Fixed KFs which are not covisible optimizable
  const int maxFixKF = 200;

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    map<KeyFrame*, tuple<int, int>> observations = (*lit)->GetObservations();
    for (map<KeyFrame*, tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) {
          lFixedKeyFrames.push_back(pKFi);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) break;
  }

  bool bNonFixed = (lFixedKeyFrames.size() == 0);

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  if (bLarge) {
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(
        1e-2);  // to avoid iterating for finding optimal lambda
    optimizer.setAlgorithm(solver);
  } else {
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e0);
    optimizer.setAlgorithm(solver);
  }

  // Set Local temporal KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local visual KeyFrame vertices
  for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                 itEnd = lpOptVisKFs.end();
       it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu)  // This should be done only for keyframe just before
                     // temporal window
    {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // TODO: Add ICP edges
  // Create ICP constraints
  vector<EdgeICP*> veICP;
  const float thHuberICP = sqrt(0.4);
  veICP.reserve(N);
  if (pbICPFlag) {
    size_t icp_count = 0;
    // 只约束当前帧和上一帧
    for (int i = 0; i < N; i++) {
      KeyFrame* pKFi = vpOptimizableKFs[i];
      if (pKFi->mnMatchesInliers > 75) continue;
      // 降采样
      Sophus::SE3d Tciw = pKFi->GetPose().cast<double>();
      g2o::Sim3 Sciw(Tciw.unit_quaternion(), Tciw.translation(), 1.0);
      Sophus::SE3d Tcjw = pKFi->mPrevKF->GetPose().cast<double>();
      g2o::Sim3 Scjw(Tcjw.unit_quaternion(), Tcjw.translation(), 1.0);
      g2o::Sim3 Scjci = Scjw * Sciw.inverse();
      Eigen::Matrix4d Tcjci;
      Tcjci.block<3, 3>(0, 0) = Scjci.rotation().toRotationMatrix();
      Tcjci.block<3, 1>(0, 3) = Scjci.translation();
      Tcjci(3, 3) = 1.;
      // cout << ": " << Tcjci << endl;
      float delat_dist =
          sqrt(Scjci.translation().x() * Scjci.translation().x() +
               Scjci.translation().y() * Scjci.translation().y());
      float delat_time = pKFi->mTimeStamp - pKFi->mPrevKF->mTimeStamp;
      // float similarity = mpVoc->score(pKFi->mPrevKF->mBowVec, pKFi->mBowVec);
      if (pKFi->mPrevKF) {
        icp_count++;
        Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
        init_T.matrix() = Tcjci;
        VertexPose* vPrevKF =
            static_cast<VertexPose*>(optimizer.vertex(pKFi->mPrevKF->mnId));
        VertexPose* vKF =
            static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if (!vPrevKF || !vKF) continue;
        RegistrationResult result = mpRegistration->RegisterPointClouds(
            *(pKFi->mPrevKF->source_points), *(pKFi->source_points), init_T);
        Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
        Eigen::Matrix4d delta_pose = relative_pose_icp * Tcjci.inverse();
        float delta_dist = sqrt(delta_pose.block<3, 1>(0, 3).x() *
                                    delta_pose.block<3, 1>(0, 3).x() +
                                delta_pose.block<3, 1>(0, 3).y() *
                                    delta_pose.block<3, 1>(0, 3).y());
        if (result.converged && result.num_inliers > 400 &&
            (result.error / result.num_inliers) < 0.01 && delta_dist < 0.1) {
          // cout << "ICP Pose In LocalInertialBA: " << endl;
          EdgeICP* eICP = new EdgeICP(relative_pose_icp.block<3, 3>(0, 0),
                                      relative_pose_icp.block<3, 1>(0, 3));
          eICP->setVertex(0, vPrevKF);
          eICP->setVertex(1, vKF);
          Eigen::Matrix<double, 6, 6> Info =
              Eigen::Matrix<double, 6, 6>::Identity();
          Info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e4;
          Info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e4;
          eICP->setInformation(Info);
          eICP->computeError();
          // std::cout << "icp relative pose error:" << eICP->error() << endl;
          optimizer.addEdge(eICP);
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          eICP->setRobustKernel(rk);
          rk->setDelta(thHuberICP);
          veICP.push_back(eICP);
        }
      }
    }
  }

  // Lidar edges
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    if (pKFi->mnMatchesInliers > 75) continue;
    vector<EdgeLidarPoint2Plane*> vpEdgesLidarPoint2Plane;
    Eigen::Matrix4d initPose = Converter::toMatrix4d(pKFi->GetPose().inverse());
    vpEdgesLidarPoint2Plane = GenerateLidarEdge<EdgeLidarPoint2Plane, KeyFrame>(
        pKFi, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        VertexPose* vKF =
            static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        edge->setVertex(0, vKF);
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;
      }
    }
  }
  vector<EdgeLidarPoint2Plane*> vpEdgesLidarPoint2Plane;
  KeyFrame* pKFi = vpOptimizableKFs[0];

  Eigen::Matrix4d initPose = Converter::toMatrix4d(pKFi->GetPose().inverse());
  vpEdgesLidarPoint2Plane = GenerateLidarEdge<EdgeLidarPoint2Plane, KeyFrame>(
      pKFi, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
  for (auto edge : vpEdgesLidarPoint2Plane) {
    if (edge) {
      Eigen::Matrix<double, 1, 1> information;
      information(0, 0) = 100.0;
      edge->setInformation(information);
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(thHuberLidar);
      edge->setRobustKernel(rk);
      VertexPose* vKF = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
      edge->setVertex(0, vKF);
      optimizer.addEdge(edge);
      edge->computeError();
      chi2Lidar += edge->chi2();
      if (edge->chi2() < 4.0) valid_edge++;
      edge_num++;
    }
  }
  // Create intertial constraints
  vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
  vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)NULL);
  vector<EdgeAccRW*> vear(N, (EdgeAccRW*)NULL);

  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // if (i == N - 1 || bRecInit) {
      // All inertial residuals are included without robust cost function,
      // but not that one linking the last optimizable keyframe inside of
      // the local window and the first fixed keyframe out. The information
      // matrix for this measurement is also downweighted. This is done to
      // avoid accumulating error due to fixing variables.
      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
      rki->setDelta(sqrt(16.0));
      // }
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);

      optimizer.addEdge(vear[i]);
    } else
      cout << "ERROR building inertial edge" << endl;
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;
  const float chi2ICP2 = 0.04;

  const unsigned long iniMPid = maxKFid * 5;

  map<int, int> mVisEdges;
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    mVisEdges[pKFi->mnId] = 0;
  }
  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    mVisEdges[(*lit)->mnId] = 0;
  }

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)
        continue;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(mit->second);

        cv::KeyPoint kpUn;

        // Monocular left observation
        if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
          mVisEdges[pKFi->mnId]++;

          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs);

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // Stereo-observation
        else if (leftIndex != -1)  // Stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          mVisEdges[pKFi->mnId]++;

          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }

        // Monocular right observation
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(mit->second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;
            mVisEdges[pKFi->mnId]++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            EdgeMono* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            // Add here uncerteinty
            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
        }
      }
    }
  }

  // cout << "Total map points: " << lLocalMapPoints.size() << endl;
  for (map<int, int>::iterator mit = mVisEdges.begin(), mend = mVisEdges.end();
       mit != mend; mit++) {
    assert(mit->second >= 3);
  }

  LOG(INFO) << "LocalInertialBA: KFs: " << N
            << ", MPs: " << lLocalMapPoints.size()
            << ", Edges: " << vpEdgesMono.size() + vpEdgesStereo.size();
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(opt_it);  // Originally to 2
  float err_end = optimizer.activeRobustChi2();
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    bool bClose = pMP->mTrackDepth < 10.f;

    if (pMP->isBad()) continue;

    if ((e->chi2() > chi2Mono2 && !bClose) ||
        (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // TODO: Some convergence problems have been detected here
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge)  // bGN)
  {
    cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
    return;
  }

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++)
    (*lit)->mnBAFixedForKF = 0;

  // Recover optimized data
  // Local temporal Keyframes
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Local visual KeyFrame
  for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                 itEnd = lpOptVisKFs.end();
       it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;
  }

  // Points
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}
Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H,
                                       const int& start, const int& end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const int c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
      svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                          svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) -
      Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg,
                                     double& scale, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, bool bMono,
                                     Eigen::MatrixXd& covInertial,
                                     bool bFixedVel, bool bGauss, float priorG,
                                     float priorA) {
  Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
  int its = 200;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  if (priorG != 0.f) solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);
    if (bFixedVel)
      VV->setFixed(true);
    else
      VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  if (bFixedVel)
    VG->setFixed(true);
  else
    VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  if (bFixedVel)
    VA->setFixed(true);
  else
    VA->setFixed(false);

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(!bMono);  // Fixed for stereo case
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  vector<pair<KeyFrame*, KeyFrame*>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());
  // std::cout << "build optimization graph" << std::endl;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (!pKFi->mpImuPreintegrated) {
        std::cout << "Not preintegrated measurement" << std::endl;
        continue;
      }

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << endl;

        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  scale = VS->estimate();

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();
  scale = VS->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
  Rwg = VGDir->estimate().Rwg;

  // Keyframes velocities and biases
  const int N = vpKFs.size();
  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();  // Velocity is scaled after
    pKFi->SetVelocity(Vw.cast<float>());

    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else
      pKFi->SetNewBias(b);
  }
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, float priorG,
                                     float priorA) {
  int its = 200;  // Check number of iterations
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);
    VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  VA->setFixed(false);

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(true);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(1.0);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(true);  // Fixed since scale is obtained from already well
                       // initialized map
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  vector<pair<KeyFrame*, KeyFrame*>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << endl;

        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

  // Keyframes velocities and biases
  const int N = vpKFs.size();
  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();
    pKFi->SetVelocity(Vw.cast<float>());

    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else
      pKFi->SetNewBias(b);
  }
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg,
                                     double& scale) {
  int its = 10;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (all variables are fixed)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + 1 + (pKFi->mnId));
    VV->setFixed(true);
    optimizer.addVertex(VV);

    // Vertex of fixed biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
    VG->setFixed(true);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
    VA->setFixed(true);
    optimizer.addVertex(VA);
  }

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(4 * (maxKFid + 1));
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(4 * (maxKFid + 1) + 1);
  VS->setFixed(false);
  optimizer.addVertex(VS);

  // Graph edges
  int count_edges = 0;
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex((maxKFid + 1) + pKFi->mnId);
      g2o::HyperGraph::Vertex* VG =
          optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VA =
          optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4 * (maxKFid + 1));
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(4 * (maxKFid + 1) + 1);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        Verbose::PrintMess(
            "Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) +
                ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " +
                to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", " +
                to_string(VGDir->id()) + ", " + to_string(VS->id()),
            Verbose::VERBOSITY_NORMAL);

        continue;
      }
      count_edges++;
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ei->setRobustKernel(rk);
      rk->setDelta(1.f);
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(its);
  optimizer.computeActiveErrors();
  float err_end = optimizer.activeRobustChi2();
  // Recover optimized data
  scale = VS->estimate();
  Rwg = VGDir->estimate().Rwg;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF,
                                      vector<KeyFrame*> vpAdjustKF,
                                      vector<KeyFrame*> vpFixedKF,
                                      bool* pbStopFlag) {
  bool bShowImages = false;

  vector<MapPoint*> vpMPs;

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;
  set<KeyFrame*> spKeyFrameBA;

  Map* pCurrentMap = pMainKF->GetMap();

  // Set fixed KeyFrame vertices
  int numInsertedPoints = 0;
  for (KeyFrame* pKFi : vpFixedKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
      Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint* pMPi : spViewMPs) {
      if (pMPi)
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
    }

    spKeyFrameBA.insert(pKFi);
  }

  // Set non fixed Keyframe vertices
  set<KeyFrame*> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
  numInsertedPoints = 0;
  for (KeyFrame* pKFi : vpAdjustKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint* pMPi : spViewMPs) {
      if (pMPi) {
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(pKFi);
  }

  const int nExpectedSize =
      (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  map<KeyFrame*, int> mpObsKFs;
  map<KeyFrame*, int> mpObsFinalKFs;
  map<MapPoint*, int> mpObsMPs;
  for (unsigned int i = 0; i < vpMPs.size(); ++i) {
    MapPoint* pMPi = vpMPs[i];
    if (pMPi->isBad()) continue;

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
    const int id = pMPi->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, tuple<int, int>> observations =
        pMPi->GetObservations();
    int nEdges = 0;
    // SET EDGES
    for (map<KeyFrame*, tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      KeyFrame* pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForMerge != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(mit->second)))
        continue;

      nEdges++;

      const cv::KeyPoint& kpUn = pKF->mvKeysUn[get<0>(mit->second)];

      if (pKF->mvuRight[get<0>(mit->second)] < 0)  // Monocular
      {
        mpObsMPs[pMPi]++;
        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber2D);

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMPi);

        mpObsKFs[pKF]++;
      } else  // RGBD or Stereo
      {
        mpObsMPs[pMPi] += 2;
        Eigen::Matrix<double, 3, 1> obs;
        const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber3D);

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMPi);

        mpObsKFs[pKF]++;
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag) bDoMore = false;

  map<unsigned long int, int> mWrongObsKF;
  if (bDoMore) {
    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);
        badMonoMP++;
      }
      e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
        badStereoMP++;
      }

      e->setRobustKernel(0);
    }
    Verbose::PrintMess("[BA]: First optimization(Huber), there are " +
                           to_string(badMonoMP) + " monocular and " +
                           to_string(badStereoMP) + " stereo bad edges",
                       Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
  set<MapPoint*> spErasedMPs;
  set<KeyFrame*> spErasedKFs;

  // Check inlier observations
  int badMonoMP = 0, badStereoMP = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badMonoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badStereoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  Verbose::PrintMess("[BA]: Second optimization, there are " +
                         to_string(badMonoMP) + " monocular and " +
                         to_string(badStereoMP) + " sterero bad edges",
                     Verbose::VERBOSITY_DEBUG);

  // Get Map Mutex
  unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }
  for (unsigned int i = 0; i < vpMPs.size(); ++i) {
    MapPoint* pMPi = vpMPs[i];
    if (pMPi->isBad()) continue;

    const map<KeyFrame*, tuple<int, int>> observations =
        pMPi->GetObservations();
    for (map<KeyFrame*, tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      KeyFrame* pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForKF != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(mit->second)))
        continue;

      if (pKF->mvuRight[get<0>(mit->second)] < 0)  // Monocular
      {
        mpObsFinalKFs[pKF]++;
      } else  // RGBD or Stereo
      {
        mpObsFinalKFs[pKF]++;
      }
    }
  }

  // Recover optimized data
  // Keyframes
  for (KeyFrame* pKFi : vpAdjustKF) {
    if (pKFi->isBad()) continue;

    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());

    int numMonoBadPoints = 0, numMonoOptPoints = 0;
    int numStereoBadPoints = 0, numStereoOptPoints = 0;
    vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
    vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];
      KeyFrame* pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        numMonoBadPoints++;
        vpMonoMPsBad.push_back(pMP);

      } else {
        numMonoOptPoints++;
        vpMonoMPsOpt.push_back(pMP);
      }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];
      KeyFrame* pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        numStereoBadPoints++;
        vpStereoMPsBad.push_back(pMP);
      } else {
        numStereoOptPoints++;
        vpStereoMPsOpt.push_back(pMP);
      }
    }

    pKFi->SetPose(Tiw);
  }

  // Points
  for (MapPoint* pMPi : vpMPs) {
    if (pMPi->isBad()) continue;

    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMPi->mnId + maxKFid + 1));
    pMPi->SetWorldPos(vPoint->estimate().cast<float>());
    pMPi->UpdateNormalAndDepth();
  }
}

void Optimizer::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF,
                                bool* pbStopFlag, Map* pMap,
                                LoopClosing::KeyFrameAndPose& corrPoses) {
  const int Nd = 6;
  const unsigned long maxKFid = pCurrKF->mnId;

  vector<KeyFrame*> vpOptimizableKFs;
  vpOptimizableKFs.reserve(2 * Nd);

  // For cov KFS, inertial parameters are not optimized
  const int maxCovKF = 30;
  vector<KeyFrame*> vpOptimizableCovKFs;
  vpOptimizableCovKFs.reserve(maxCovKF);

  // Add sliding window for current KF
  vpOptimizableKFs.push_back(pCurrKF);
  pCurrKF->mnBALocalForKF = pCurrKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else
      break;
  }

  list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
  } else {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Add temporal neighbours to merge KF (previous and next KFs)
  vpOptimizableKFs.push_back(pMergeKF);
  pMergeKF->mnBALocalForKF = pCurrKF->mnId;

  // Previous KFs
  for (int i = 1; i < (Nd / 2); i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else
      break;
  }

  // We fix just once the old map
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Next KFs
  if (pMergeKF->mNextKF) {
    vpOptimizableKFs.push_back(pMergeKF->mNextKF);
    vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
  }

  while (vpOptimizableKFs.size() < (2 * Nd)) {
    if (vpOptimizableKFs.back()->mNextKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else
      break;
  }

  int N = vpOptimizableKFs.size();

  // Optimizable points seen by optimizable keyframes
  list<MapPoint*> lLocalMapPoints;
  map<MapPoint*, int> mLocalObs;
  for (int i = 0; i < N; i++) {
    vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      // Using mnBALocalForKF we avoid redundance here, one MP can not be
      // added several times to lLocalMapPoints
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pCurrKF->mnId) {
            mLocalObs[pMP] = 1;
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pCurrKF->mnId;
          } else {
            mLocalObs[pMP]++;
          }
    }
  }

  std::vector<std::pair<MapPoint*, int>> pairs;
  pairs.reserve(mLocalObs.size());
  for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
    pairs.push_back(*itr);
  sort(pairs.begin(), pairs.end(), sortByVal);

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
  // Local Keyframes
  int i = 0;
  for (vector<pair<MapPoint*, int>>::iterator lit = pairs.begin(),
                                              lend = pairs.end();
       lit != lend; lit++, i++) {
    map<KeyFrame*, tuple<int, int>> observations =
        lit->first->GetObservations();
    if (i >= maxCovKF) break;
    for (map<KeyFrame*, tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
          pKFi->mnBAFixedForKF !=
              pCurrKF->mnId)  // If optimizable or already included...
      {
        pKFi->mnBALocalForKF = pCurrKF->mnId;
        if (!pKFi->isBad()) {
          vpOptimizableCovKFs.push_back(pKFi);
          break;
        }
      }
    }
  }

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  // Set Local KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local cov keyframes vertices
  int Ncov = vpOptimizableCovKFs.size();
  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // Create intertial constraints
  vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
  vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)NULL);
  vector<EdgeAccRW*> vear(N, (EdgeAccRW*)NULL);
  for (int i = 0; i < N; i++) {
    // cout << "inserting inertial edge " << i << endl;
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // TODO Uncomment
      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      rki->setDelta(sqrt(16.92));
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);
      optimizer.addEdge(vear[i]);
    } else
      Verbose::PrintMess("ERROR building inertial edge",
                         Verbose::VERBOSITY_NORMAL);
  }

  Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

  // Set MapPoint vertices
  const int nExpectedSize =
      (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    if (!pMP) continue;

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (map<KeyFrame*, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi) continue;

      if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
          (pKFi->mnBAFixedForKF != pCurrKF->mnId))
        continue;

      if (pKFi->mnId > maxKFid) {
        continue;
      }

      if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
        continue;

      if (!pKFi->isBad()) {
        const cv::KeyPoint& kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

        if (pKFi->mvuRight[get<0>(mit->second)] < 0)  // Monocular observation
        {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);
          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        } else  // stereo observation
        {
          const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }
      }
    }
  }

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(8);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Mono2) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);
  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Points
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}
int Optimizer::PoseInertialOptimizationLastKeyFrame(
    Frame* pFrame, bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError, const int nIterations) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setVerbose(false);
  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  KeyFrame* pKF = pFrame->mpLastKeyFrame;
  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(6.0);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < nIterations; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];
    float avgReprojectionError = 0.0f;
    float chi2IMU = ei->chi2();
    if (chi2IMU > 15.0) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(2.0);
      ei->setRobustKernel(rk);
      ei->setInformation(ei->information() * 1e-2);
    }
    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // For stereo observations
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);  // not included in next optimization
        nBadStereo++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;
    avgReprojectionError /= nInliers;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize keyFframe states and generate new prior for
  // frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();
  H.block<3, 3>(9, 9) += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  pFrame->mpcpi =
      new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                            VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}

int Optimizer::PoseLidarVisualInertialOptimizationLastKeyFrame(
    Frame* pFrame, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError, const int nIterations, int& nInliersLidar,
    float& residual) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setVerbose(false);
  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  KeyFrame* pKF = pFrame->mpLastKeyFrame;
  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(6.0);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);

  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  // Lidar
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  Eigen::Matrix4d initPose = Converter::toMatrix4d(pFrame->GetPose().inverse());
  vector<EdgeLidarPoint2Plane*> vpEdgesLidarPoint2Plane;
  // vpEdgesLidarPoint2Plane.assign(pFrame->mpPointCloudDownsampled->size(),
  //                                nullptr);
  // vpEdgesLidarPoint2Plane.reserve(2000);
  // vector<bool> vpEdgesLidarPoint2PlaneOutlier(
  //     pFrame->mpPointCloudDownsampled->size(), false);
  for (size_t it = 0; it < nIterations; it++) {
    vpEdgesLidarPoint2Plane.clear();
    vpEdgesLidarPoint2Plane = GenerateLidarEdge<EdgeLidarPoint2Plane, Frame>(
        pFrame, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    // vpEdgesLidarPoint2PlaneOutlier.clear();
    // vpEdgesLidarPoint2PlaneOutlier.assign(vpEdgesLidarPoint2Plane.size(),
    //                                       false);
    edge_num = 0;
    chi2Lidar = 0;
    valid_edge = 0;
    float chi2IMU = ei->chi2();
    if (chi2IMU > 15.0) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(2.0);
      ei->setRobustKernel(rk);
      ei->setInformation(ei->information() * 1e-2);
    }
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        if (abs(pFrame->mTimeStamp - pKF->mTimeStamp > 0.8)) {
          edge->setInformation(information * 1e2);
        }
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, VP);
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;

        // edge->computeError();
        // chi2Lidar += edge->chi2();
        // if (edge->chi2() < 1.0) valid_edge++;
        // edge_num++;
        // edge->computeError();
        // chi2Lidar += edge->chi2();
        // if (edge->chi2() < 1.0) {
        //   valid_edge++;
        //   edge->setLevel(0);
        // } else {
        //   vpEdgesLidarPoint2PlaneOutlier[edge_num] = true;
        //   edge->setLevel(1);
        // }
        // if (it == nIterations - 2) edge->setRobustKernel(0);
      }
    }

    if (edge_num != 0) chi2Lidar /= edge_num;

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    nInliersLidar = valid_edge;
    residual = chi2Lidar;
    Sophus::SE3f Twb(VP->estimate().Rwb.cast<float>(),
                     VP->estimate().twb.cast<float>());
    Sophus::SE3f Tbw = Twb.inverse();

    Sophus::SE3f Tcw = pFrame->mImuCalib.mTcb * Tbw;
    initPose = Converter::toMatrix4d(Tcw.inverse());
    float avgReprojectionError = 0.0f;
    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // For stereo observations
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);  // not included in next optimization
        nBadStereo++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;
    avgReprojectionError /= nInliers;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }

    if (optimizer.edges().size() < 10) {
      break;
    }

    // 必须放大最后，不然可能提前break
    if (it < nIterations - 1) {
      for (auto edge : vpEdgesLidarPoint2Plane) {
        if (edge) optimizer.removeEdge(edge);
      }
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize keyFrame states and generate new prior for
  // frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();
  H.block<3, 3>(9, 9) += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }
  for (auto edge : vpEdgesLidarPoint2Plane) {
    if (edge) {
      H.block<6, 6>(0, 0) += edge->GetHessian();
      // delete edge;
    }
  }
  // for (size_t i = 0, iend = vpEdgesLidarPoint2Plane.size(); i < iend; i++) {
  //   if (!vpEdgesLidarPoint2PlaneOutlier[i] && vpEdgesLidarPoint2Plane[i]) {
  //     H.block<6, 6>(15, 15) += vpEdgesLidarPoint2Plane[i]->GetHessian();
  //   }
  // }
  pFrame->mpcpi =
      new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                            VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(
    Frame* pFrame, bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError, const int nIterations) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(6.0);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (!pFp->mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < nIterations; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        avgReprojectionError += chi2;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;
    avgReprojectionError /= nInliers;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  H = Marginalize(H, 0, 14);
  pFrame->mpcpi = new ConstraintPoseImu(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}

int Optimizer::PoseLidarVisualInertialOptimizationLastFrame(
    Frame* pFrame, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError, const int nIterations, int& nInliersLidar,
    float& residual) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(6.0);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (!pFp->mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;

  // Lidar
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

  //
  Eigen::Matrix4d initPose = Converter::toMatrix4d(pFrame->GetPose().inverse());
  initPose.block<3, 1>(0, 3) += Eigen::Vector3d(0.1, 0.1, 0.1);
  vector<EdgeLidarPoint2Plane*> vpEdgesLidarPoint2Plane;
  vpEdgesLidarPoint2Plane.assign(pFrame->mpPointCloudDownsampled->size(),
                                 nullptr);
  // vector<bool> vpEdgesLidarPoint2PlaneOutlier(
  //     pFrame->mpPointCloudDownsampled->size(), false);
  // vpEdgesLidarPoint2Plane.reserve(2000);
  for (size_t it = 0; it < nIterations; it++) {
    vpEdgesLidarPoint2Plane.clear();
    vpEdgesLidarPoint2Plane = GenerateLidarEdge<EdgeLidarPoint2Plane, Frame>(
        pFrame, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    // vpEdgesLidarPoint2PlaneOutlier.clear();
    // vpEdgesLidarPoint2PlaneOutlier.assign(vpEdgesLidarPoint2Plane.size(),
    //                                       false);
    edge_num = 0;
    chi2Lidar = 0;
    valid_edge = 0;
    float chi2IMU = ei->chi2();
    if (chi2IMU > 15.0) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(2.0);
      ei->setRobustKernel(rk);
      ei->setInformation(ei->information() * 1e-2);
    }
    for (auto edge : vpEdgesLidarPoint2Plane) {
      // TODO: Problem with the edge
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, VP);
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;
        // if (edge->chi2() < 1.0) {
        //   valid_edge++;
        //   edge->setLevel(0);
        // } else {
        //   vpEdgesLidarPoint2PlaneOutlier[edge_num] = true;
        //   edge->setLevel(1);
        // }
      }
    }

    if (edge_num != 0) chi2Lidar /= edge_num;

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];
    nInliersLidar = valid_edge;
    residual = chi2Lidar;
    Sophus::SE3f Twb(VP->estimate().Rwb.cast<float>(),
                     VP->estimate().twb.cast<float>());
    Sophus::SE3f Tbw = Twb.inverse();

    Sophus::SE3f Tcw = pFrame->mImuCalib.mTcb * Tbw;
    initPose = Converter::toMatrix4d(Tcw.inverse());

    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        avgReprojectionError += chi2;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == nIterations - 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == nIterations - 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;
    avgReprojectionError /= nInliers;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }

    // some bug here
    if (optimizer.edges().size() < 10) {
      break;
    }

    // Remove Lidar edges in the end
    if (it < nIterations - 1) {
      for (auto edge : vpEdgesLidarPoint2Plane) {
        if (edge) optimizer.removeEdge(edge);
      }
    }
  }

  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }
  for (auto edge : vpEdgesLidarPoint2Plane) {
    if (edge) {
      H.block<6, 6>(15, 15) += edge->GetHessian();
    }
  }

  // LOG(WARNING) << "Visual Total inliers: " << tot_in
  //              << " Total outliers: " << tot_out;
  // LOG(WARNING) << "Lidar Total inliers: " << valid_edge
  //              << " Total residual: " << chi2Lidar;
  // for (size_t i = 0, iend = vpEdgesLidarPoint2Plane.size(); i < iend; i++) {
  //   if (!vpEdgesLidarPoint2PlaneOutlier[i] && vpEdgesLidarPoint2Plane[i]) {
  //     H.block<6, 6>(15, 15) += vpEdgesLidarPoint2Plane[i]->GetHessian();
  //   }
  // }
  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = new ConstraintPoseImu(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}

void Optimizer::pointAssociateToMap(
    PointType const* const pi, PointType* const po,
    const Eigen::Matrix4d& transPointAssociateToMap) {
  po->x = transPointAssociateToMap(0, 0) * pi->x +
          transPointAssociateToMap(0, 1) * pi->y +
          transPointAssociateToMap(0, 2) * pi->z +
          transPointAssociateToMap(0, 3);
  po->y = transPointAssociateToMap(1, 0) * pi->x +
          transPointAssociateToMap(1, 1) * pi->y +
          transPointAssociateToMap(1, 2) * pi->z +
          transPointAssociateToMap(1, 3);
  po->z = transPointAssociateToMap(2, 0) * pi->x +
          transPointAssociateToMap(2, 1) * pi->y +
          transPointAssociateToMap(2, 2) * pi->z +
          transPointAssociateToMap(2, 3);
  po->rgb = pi->rgb;
}

int Optimizer::PoseLidarVisualOptimization(
    Frame* pFrame, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    const bool bFrame2FrameReprojError, const bool bFrame2MapReprojError,
    const int nIterations, int& nLidarInliers, float& residual) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float> Tcw = pFrame->GetPose();
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                 Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;
  float reprojectionError = 100.0f;

  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
  vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815);
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Conventional SLAM
        if (!pFrame->mpCamera2) {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0) {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else  // Stereo observation
          {

            // if (pFrame->mvbOutlier[i]) continue;
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            const float& kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
        // SLAM with respect a rigid body
        else {
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;

          if (i < pFrame->Nleft) {  // Left camera observation
            kpUn = pFrame->mvKeys[i];
            if (pFrame->mvbOutlier[i]) continue;
            // pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            if (pFrame->mvbOutlier[i]) continue;
            // pFrame->mvbOutlier[i] = false;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera2;
            e->Xw = pMP->GetWorldPos().cast<double>();

            e->mTrl = g2o::SE3Quat(
                pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
                pFrame->GetRelativePoseTrl().translation().cast<double>());

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
  }

  if (nInitialCorrespondences < 3) return 0;
  // We perform 4 optimizations, after each optimization we classify
  // observation
  // as inlier/outlier At the next optimization, outliers are not included,
  // but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  // const float chi2Stereo[4] = {7.815, 6.815, 5.815, 5.0};
  const int its[4] = {10, 5, 5, 5};
  int nBad = 0;
  int nGood = 0;

  // For Lidar Point to Plane
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  Eigen::Matrix4d initPose = Converter::toMatrix4d(pFrame->GetPose().inverse());
  // initPose.block<3, 1>(0, 3) += Eigen::Vector3d(0.1, 0.1, 0.1);
  for (size_t it = 0; it < nIterations; it++) {
    vector<EdgeSE3LidarPoint2Plane*> vpEdgesLidarPoint2Plane =
        GenerateLidarEdge<EdgeSE3LidarPoint2Plane, Frame>(
            pFrame, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    edge_num = 0;
    chi2Lidar = 0;
    valid_edge = 0;
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(0)));
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;
      }
    }

    if (edge_num != 0) {
      chi2Lidar /= edge_num;
    } else {
      // cout << "No valid edge" << endl;
      continue;
    }
    LOG(WARNING) << "Iteration " << it << " Lidar Chi2: " << chi2Lidar;
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) optimizer.removeEdge(edge);
    }
    nLidarInliers = valid_edge;
    residual = chi2Lidar;
    g2o::VertexSE3Expmap* vSE3_recov =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                            SE3quat_recov.translation().cast<float>());
    initPose = Converter::toMatrix4d(pose.inverse());

    nBad = 0;
    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      reprojectionError = chi2;

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nGood++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

      const size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      // 验后重投影误差
      const float chi2 = e->chi2();
      reprojectionError = chi2;
      // avgReprojectionError += chi2;

      if (chi2 > chi2Stereo[it]) {
        // 最后一次标记为outlier
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        nGood++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    avgReprojectionError /= nGood;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    if (optimizer.edges().size() < 10) break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                          SE3quat_recov.translation().cast<float>());
  pFrame->SetPose(pose);
  return nInitialCorrespondences - nBad;
}
void Optimizer::PoseLidarOptimization(
    Frame* pFrame, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    const int nIterations, int& nLidarInliers, float& residual) {
  // Set Current Frame vertex
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float> Tcw = pFrame->GetPose();
  // 平移施加4cm噪声
  Tcw.translation() += Eigen::Vector3f(0.1, 0.1, 0.1);
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                 Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  LOG(WARNING) << "Frame ID " << pFrame->mnId;
  // Eigen::Matrix4d initPose =
  // Converter::toMatrix4d(pFrame->GetPose().inverse());
  Eigen::Matrix4d initPose = Converter::toMatrix4d(Tcw.inverse());
  cout << "Input Cloud Size:" << pFrame->mpPointCloudDownsampled->size()
       << endl;
  for (size_t it = 0; it < nIterations; it++) {
    vector<EdgeSE3LidarPoint2Plane*> vpEdgesLidarPoint2Plane =
        GenerateLidarEdge<EdgeSE3LidarPoint2Plane, Frame>(
            pFrame, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    if (vpEdgesLidarPoint2Plane.size() == 0) continue;
    edge_num = 0;
    chi2Lidar = 0;
    valid_edge = 0;
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(0)));
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 4.0) valid_edge++;
        edge_num++;
      }
    }

    if (edge_num != 0) {
      chi2Lidar /= edge_num;
    } else {
      // cout << "No valid edge" << endl;
      continue;
    }
    LOG(WARNING) << "Iteration " << it << " Lidar Chi2: " << chi2Lidar;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) optimizer.removeEdge(edge);
    }
    nLidarInliers = valid_edge;
    residual = chi2Lidar;

    g2o::VertexSE3Expmap* vSE3_recov =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                            SE3quat_recov.translation().cast<float>());
    initPose = Converter::toMatrix4d(pose.inverse());
  }
  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                          SE3quat_recov.translation().cast<float>());
  if (valid_edge > 50 && chi2Lidar < 1.0) {
    pFrame->SetPose(pose);
  } else {
    LOG(WARNING) << "Lidar Pose Optimization Failed";
    LOG(WARNING) << "Valid Edge: " << valid_edge << " Chi2: " << chi2Lidar;
  }
}

void Optimizer::PoseLidarInertialOptimizationLastFrame(
    Frame* pFrame, pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError, const int nIterations, int& nLidarInliers,
    float& residual) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(6.0);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (!pFp->pre_mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->pre_mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  const int its[4] = {10, 10, 10, 10};

  // Lidar
  const float thHuberLidar = sqrt(1.0);
  float chi2Lidar = 0;
  size_t valid_edge = 0;
  size_t edge_num = 0;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
  Eigen::Matrix4d initPose = Converter::toMatrix4d(pFrame->GetPose().inverse());
  for (size_t it = 0; it < nIterations; it++) {
    vector<EdgeLidarPoint2Plane*> vpEdgesLidarPoint2Plane =
        GenerateLidarEdge<EdgeLidarPoint2Plane, Frame>(
            pFrame, initPose, laserCloudSurfFromMapDS, kdtreeSurfFromMap);
    edge_num = 0;
    chi2Lidar = 0;
    valid_edge = 0;

    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) {
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1e2;
        edge->setInformation(information);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuberLidar);
        edge->setRobustKernel(rk);
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(0)));
        optimizer.addEdge(edge);
        edge->computeError();
        chi2Lidar += edge->chi2();
        if (edge->chi2() < 1.0) valid_edge++;
        edge_num++;
      }
    }

    if (edge_num != 0) {
      chi2Lidar /= edge_num;
    } else {
      // cout << "No valid edge" << endl;
      continue;
    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    for (auto edge : vpEdgesLidarPoint2Plane) {
      if (edge) optimizer.removeEdge(edge);
    }
    nLidarInliers = valid_edge;
    residual = chi2Lidar;
    Sophus::SE3f Twb(VP->estimate().Rwb.cast<float>(),
                     VP->estimate().twb.cast<float>());
    Sophus::SE3f Tbw = Twb.inverse();

    Sophus::SE3f Tcw = pFrame->mImuCalib.mTcb * Tbw;

    initPose = Converter::toMatrix4d(Tcw.inverse());
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);
}

template <typename EdgeType, typename FrameType>
vector<EdgeType*> Optimizer::GenerateLidarEdge(
    FrameType* pFrame, Eigen::Matrix4d initPose,
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS,
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap) {
  if (!pFrame->mpPointCloudDownsampled ||
      pFrame->mpPointCloudDownsampled->size() < 50)
    return vector<EdgeType*>();
  size_t laserCloudGroundLastDSNum = pFrame->mpPointCloudDownsampled->size();
  std::vector<EdgeType*> vpEdgesLidarPoint2Plane;
  vpEdgesLidarPoint2Plane.resize(laserCloudGroundLastDSNum, nullptr);
  // 预先分配矩阵，避免在循环中动态分配内存
  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;
  matA0.setZero();
  matB0.fill(-1);
  matX0.setZero();
#pragma omp parallel for schedule(dynamic) private(matA0, matX0)
  for (int i = 0; i < laserCloudGroundLastDSNum; i++) {
    PointType pointOri, pointSel, coeff;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = pFrame->mpPointCloudDownsampled->points[i];
    pointAssociateToMap(&pointOri, &pointSel, initPose);
    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                      pointSearchSqDis);
    // TODO：Use Ivoxel to find the nearest points


    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
        matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
        matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
      }

      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                 pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                 pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                 pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      // 平面合格
      if (planeValid) {
        // 当前激光帧点到平面距离
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointSel.x * pointSel.x +
                                    pointSel.y * pointSel.y +
                                    pointSel.z * pointSel.z));

        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        Eigen::Vector4d plane(pa, pb, pc, pd);

        if (s > 0.1) {
          EdgeType* edge = new EdgeType(curr_point, plane, s);
          vpEdgesLidarPoint2Plane[i] = edge;
        }
      }
    }
  }
  return vpEdgesLidarPoint2Plane;
}

int Optimizer::PoseInertialICPOptimizationLastFrame(
    Frame* pFrame, bool bRecInit, const bool bFrame2FrameReprojError,
    const bool bFrame2MapReprojError) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Vector3f translation(0.0f, 0.0f, 0.0f);
  Sophus::SE3f se3_rt(rotation, translation);
  pFrame->mImuCalib.mTcb = se3_rt;
  Eigen::Matrix4f current_rotation = pFrame->GetPose().matrix();
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);
  const float thHuberICP = sqrt(0.4);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // ICP edge
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d dt = Eigen::Vector3d::Zero();
  pFrame->GetICPDeltaPose(dR, dt);  // Get the delta pose from ICP
  EdgeICP* eicp = new EdgeICP(dR, dt);
  eicp->setVertex(0, VPk);
  eicp->setVertex(1, VP);
  eicp->computeError();
  // cout << "before optimization icp pose error:" << ei->error() << endl;
  // // 位移精度为0.05m 姿态精度为1度，根据精度设置信息矩阵
  Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
  Info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  eicp->setInformation(Info);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  eicp->setRobustKernel(rk);
  rk->setDelta(thHuberICP);
  optimizer.addEdge(eicp);

  if (!pFp->mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  float reprojectionError = 100.0f;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];
    eicp->computeError();
    const float icp_chi2 = eicp->chi2();
    if (icp_chi2 > 5.0) {
      eicp->setLevel(1);
    } else {
      eicp->setLevel(0);
    }
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      reprojectionError = chi2;
      // pFrame->SetReprojectionError(chi2);
      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }
    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }
      const float chi2 = e->chi2();
      reprojectionError = chi2;
      // pFrame->SetReprojectionError(chi2);
      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        avgReprojectionError += chi2;
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;
    avgReprojectionError /= nInliers;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  if (static_cast<double>(nInitialCorrespondences - nBad) /
              nInitialCorrespondences >
          0.6 &&
      reprojectionError < 5.0) {
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                               VP->estimate().twb.cast<float>(),
                               VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);
  }
  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();
  Eigen::Matrix<double, 6, 12> J;
  J.setZero();
  J = eicp->GetJacobian();
  H.block<24, 24>(0, 0) += ei->GetHessian();
  // 对应上一帧旋转平移
  H.block<6, 6>(0, 0) += eicp->GetHessian1();
  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();
  // 两帧位姿间的hessian
  H.block<6, 6>(0, 15) += J.block<6, 6>(0, 0).transpose() * J.block<6, 6>(0, 6);
  H.block<6, 6>(15, 0) += J.block<6, 6>(0, 6).transpose() * J.block<6, 6>(0, 0);
  // 对应当前帧旋转平移
  H.block<6, 6>(15, 15) += eicp->GetHessian2();
  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = new ConstraintPoseImu(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}

int Optimizer::PoseICPOptimizationLastFrame(Frame* pFrame, bool bRecInit,
                                            const bool bFrame2FrameReprojError,
                                            const bool bFrame2MapReprojError) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Vector3f translation(0.0f, 0.0f, 0.0f);
  Sophus::SE3f se3_rt(rotation, translation);
  pFrame->mImuCalib.mTcb = se3_rt;
  Eigen::Matrix4f current_rotation = pFrame->GetPose().matrix();
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);

  // Set MapPoint vertices
  // cout << "Set MapPoint vertices" << endl;
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);
  float reprojectionError = 100.0f;
  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);
  const float thHuberICP = sqrt(1.0);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          // if (pFrame->mvbOutlier[i]) continue;
          nInitialCorrespondences++;
          pFrame->mvbOutlier[i] = false;
          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;
  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(1);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d dt = Eigen::Vector3d::Zero();
  pFrame->GetICPDeltaPose(dR, dt);  // Get the delta pose from ICP
  EdgeICP* ei = new EdgeICP(dR, dt);
  ei->setVertex(0, VPk);
  ei->setVertex(1, VP);
  ei->computeError();
  // 位移精度为0.05m 姿态精度为1度，根据精度设置信息矩阵
  Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
  Info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e8;
  Info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e8;
  ei->setInformation(Info);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  ei->setRobustKernel(rk);
  rk->setDelta(thHuberICP);

  optimizer.addEdge(ei);
  Eigen::Matrix<double, 6, 6> PriorHession;
  EdgePriorPoseICP* ep = NULL;
  if (!pFp->mpcp_icp) {
    // cout << "pFp->mpcp_icp does not exist!!!\nPrevious Frame \n";
    Verbose::PrintMess("pFp->mpcp_icp does not exist!!!\nPrevious Frame " +
                           to_string(pFp->mnId),
                       Verbose::VERBOSITY_NORMAL);
  } else {
    // 边缘化先验位姿
    ep = new EdgePriorPoseICP(pFp->mpcp_icp);
    ep->setVertex(0, VPk);
    g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
    ep->setRobustKernel(rkp);
    rkp->setDelta(5);
    optimizer.addEdge(ep);
  }
  // We perform 4 optimizations, after each optimization we classify
  // observation as inlier/outlier At the next optimization, outliers are not
  // included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  // const float chi2Stereo[4] = {7.815, 6.815, 5.815, 5.0};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];
    ei->computeError();
    const float icp_chi2 = ei->chi2();
    if (icp_chi2 > 5.0) {
      ei->setLevel(1);
    } else {
      ei->setLevel(0);
    }
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    float avgReprojectionError = 0.0f;
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      reprojectionError = chi2;
      // avgReprojectionError += chi2;

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        // if (it == 3) pFrame->mvbOutlier[idx] = true;
        // 缩小信息矩阵
        // float scale = chi2 / chi2Stereo[it] * 2.5;
        // Eigen::Matrix3d InfoMatrix = e->information() * scale;
        // e->setInformation(InfoMatrix);
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        avgReprojectionError += chi2;
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    avgReprojectionError /= nInliersStereo;
    if (bFrame2FrameReprojError)
      pFrame->SetFrame2FrameReprojError(avgReprojectionError);
    if (bFrame2MapReprojError) {
      pFrame->SetFrame2MapReprojError(avgReprojectionError);
    }
    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // ei->computeError();
  // cout << "after optimization icp pose error:" << ei->error() << endl;
  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 12, 12> H;
  Eigen::Matrix<double, 6, 12> J;
  H.setZero();
  J.setZero();
  J = ei->GetJacobian();
  H.block<6, 6>(0, 0) += ei->GetHessian1();
  if (pFp->mpcp_icp && ep) {
    H.block<6, 6>(0, 0) += ep->GetHessian();
  }
  H.block<6, 6>(6, 6) += ei->GetHessian2();

  H.block<6, 6>(0, 6) += J.block<6, 6>(0, 0).transpose() * J.block<6, 6>(0, 6);

  H.block<6, 6>(6, 0) += J.block<6, 6>(0, 6).transpose() * J.block<6, 6>(0, 0);

  int tot_in = 0, tot_out = 0;

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      // 仅针对当前帧
      H.block<6, 6>(6, 6) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  // 边缘化上一帧的状态
  H = Marginalize(H, 0, 5);
  Eigen::Matrix3d Rwc = VP->estimate().Rcw[0].transpose();
  Eigen::Vector3d twc = -Rwc * VP->estimate().tcw[0];
  pFrame->mpcp_icp = new ConstraintPoseICP(Rwc, twc, H.block<6, 6>(6, 6));
  delete pFp->mpcp_icp;
  pFp->mpcp_icp = NULL;
  if (static_cast<double>(nInitialCorrespondences - nBad) /
              nInitialCorrespondences >
          0.6 &&
      reprojectionError < 5.0) {
    pFrame->SetPose(VP->estimate().Rcw[0].cast<float>(),
                    VP->estimate().tcw[0].cast<float>());
  }
  return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(
    Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<KeyFrame*, set<KeyFrame*>>& LoopConnections,
    const bool& bUseICPConstraint) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4;

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolverX::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);

  vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);

  // ICP
  std::shared_ptr<RegistrationGICP> mpRegistration =
      std::make_shared<RegistrationGICP>();
  const int minFeat = 100;
  // Set KeyFrame vertices
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;

    VertexPose4DoF* V4DoF;

    const int nIDi = pKF->mnId;

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      const g2o::Sim3 Swc = it->second.inverse();
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      Eigen::Vector3d twc = Swc.translation();
      V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      V4DoF = new VertexPose4DoF(pKF);
    }

    if (pKF == pLoopKF) V4DoF->setFixed(true);

    V4DoF->setId(nIDi);
    V4DoF->setMarginalized(false);

    optimizer.addVertex(V4DoF);
    vpVertices[nIDi] = V4DoF;
  }
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses
  // are just in 4DoF
  Eigen::Matrix<double, 6, 6> matLambda =
      Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;
  matLambda(0, 0) = 1e3;

  // Set Loop edges
  Edge4DoF* e_loop;
  for (map<KeyFrame*, set<KeyFrame*>>::const_iterator
           mit = LoopConnections.begin(),
           mend = LoopConnections.end();
       mit != mend; mit++) {
    KeyFrame* pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const set<KeyFrame*>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];

    for (set<KeyFrame*>::const_iterator sit = spConnections.begin(),
                                        send = spConnections.end();
         sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(*sit) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(1.0);
      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));

      e->information() = matLambda;
      e->setRobustKernel(rk);
      e_loop = e;
      optimizer.addEdge(e);

      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // 1. Set normal edges
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    KeyFrame* pKF = vpKFs[i];

    const int nIDi = pKF->mnId;

    g2o::Sim3 Siw;

    // Use noncorrected poses for posegraph edges
    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // 1.1.0 Spanning tree edge
    KeyFrame* pParentKF = static_cast<KeyFrame*>(NULL);
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    KeyFrame* prevKF = pKF->mPrevKF;
    if (prevKF) {
      int nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(prevKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.2 Loop edges
    const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
    for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      KeyFrame* pLKF = *sit;
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Swl = itl->second.inverse();
        else
          Swl = vScw[pLKF->mnId].inverse();

        g2o::Sim3 Sil = Siw * Swl;
        Eigen::Matrix4d Til;
        Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
        Til.block<3, 1>(0, 3) = Sil.translation();
        Til(3, 3) = 1.;

        Edge4DoF* e = new Edge4DoF(Til);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pLKF->mnId)));
        e->information() = matLambda;
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(1.0);
        e->setRobustKernel(rk);
        optimizer.addEdge(e);

        // TODO: Add ICP edges
        if (bUseICPConstraint) {
          Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
          // 回环帧到当前帧的变换
          init_T.matrix() = Til.cast<double>();
          RegistrationResult result = mpRegistration->RegisterPointClouds(
              *(pKF->source_points), *(pLKF->source_points), init_T);
          Eigen::Matrix4d relative_pose_icp = result.T_target_source.matrix();
          if (result.converged && result.num_inliers > 100 &&
              (result.error / result.num_inliers) < 0.1) {
            cout << "ICP edge added" << endl;
            cout << "result.error: " << result.error << endl;
            cout << "Initial T: " << init_T.matrix() << endl;
            cout << "relative_pose_icp: " << relative_pose_icp << endl;
            Edge4DoF* eicp = new Edge4DoF(relative_pose_icp);
            eicp->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(nIDi)));
            eicp->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pLKF->mnId)));
            eicp->information() = matLambda;
            optimizer.addEdge(eicp);
          }
        }
      }
    }

    // 1.3 Covisibility graph edges
    const vector<KeyFrame*> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      KeyFrame* pKFn = *vit;
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF &&
          !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Swn;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Swn = itn->second.inverse();
          else
            Swn = vScw[pKFn->mnId].inverse();

          g2o::Sim3 Sin = Siw * Swn;
          Eigen::Matrix4d Tin;
          Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
          Tin.block<3, 1>(0, 3) = Sin.translation();
          Tin(3, 3) = 1.;
          Edge4DoF* e = new Edge4DoF(Tin);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(nIDi)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFn->mnId)));
          e->information() = matLambda;
          optimizer.addEdge(e);


        }
      }
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];

    const int nIDi = pKFi->mnId;

    VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;

    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
    nIDr = pRefKF->mnId;

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }
  pMap->IncreaseChangeIndex();
}
bool Optimizer::InertialOptimization(Map* pMap, Eigen::Vector3d& bg,
                                     float priorG) {
  int its = 20;  // Check number of iterations
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  /*
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType * linearSolver;

  linearSolver = new
  g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new
  g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  */

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.back());
  VG->setId(0);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  int cnt = 0;
  for (int i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF1 = vpKFs[i]->mPrevKF;
    KeyFrame* pKF2 = vpKFs[i];

    if (!pKF1 || !pKF2) {
      cnt += 1;
      continue;
    }

    IMU::Preintegrated* pInt12 = pKF2->mpImuPreintegrated;
    if (!pInt12) {
      cnt += 1;
      continue;
    }

    // pKF2->mpImuPreintegrated->SetNewBias(pKF1->GetImuBias());

    // g2o::HyperGraph::Vertex* VG = optimizer.vertex(1);

    EdgeGyro* ei = new EdgeGyro(pInt12, pKF1->GetImuRotation().cast<double>(),
                                pKF2->GetImuRotation().cast<double>());
    ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));

    optimizer.addEdge(ei);
  }

  if (cnt > (vpKFs.size() - 3)) return false;

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(its);
  optimizer.computeActiveErrors();
  float err_end = optimizer.activeRobustChi2();

  // TODO: Some convergence problems have been detected here
  if (2 * err < err_end || isnan(err) || isnan(err_end)) {
    cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
    bg.setZero();
    return false;
  }

  bg << VG->estimate();
  return true;
}

}  // namespace ORB_SLAM3

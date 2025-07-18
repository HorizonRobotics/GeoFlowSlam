// Author of SSL_SLAM2: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_

// define lidar parameter
//  opencv
#include <opencv2/opencv.hpp>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;
namespace ORB_SLAM3 {

class CommonParam {
 public:
  CommonParam() {};
  void loadParam(std::string& path);
  int getCoreNum(void);
  int getInitFrame(void);
  int getNearbyFrame(void);
  Eigen::Isometry3d getTbl(void);

 private:
  int core_num;
  int init_frame;
  int nearby_frame;
  Eigen::Isometry3d Tbl;
};

// define lidar parameter
class LidarParam {
 public:
  LidarParam() {};
  void loadParam(std::string& path);
  int getFrequency(void);
  double getMinDistance(void);
  double getMaxDistance(void);
  double getHorizontalAngle(void);
  double getMapResolution(void);
  Eigen::Matrix<double, 6, 1> getOdomN(void);
  double getEdgeN();
  double getSurfN();
  double getLocalMapSize(void);
  double getLocalMapResolution(void);
  double getMapCellWidth(void);
  double getMapCellHeight(void);
  double getMapCellDepth(void);
  int getMapCellWidthRange(void);
  int getMapCellHeightRange(void);
  int getMapCellDepthRange(void);

 private:
  int frequency;
  double min_distance;
  double max_distance;
  double horizontal_angle;
  double map_resolution;
  Eigen::Matrix<double, 6, 1> odom_n;
  double edge_n;
  double surf_n;
  double local_map_resolution;
  double local_map_size;
  double map_cell_width;
  double map_cell_height;
  double map_cell_depth;
  int map_cell_width_range;
  int map_cell_height_range;
  int map_cell_depth_range;
};

}  // namespace ORB_SLAM3

#endif  // _LIDAR_H_

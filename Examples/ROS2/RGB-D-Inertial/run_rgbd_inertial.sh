
source ./install/setup.bash &&
ros2 run orbslam3 rgbd_inertial_ros2 \
 ../../../Vocabulary/ORBvoc.txt ./config/go2_op_icp_lidar_outdoor.yaml \
 /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_45_37
# 调用PoseEvaluator.py进行姿态评估
# ./PoseEvaluator.py: Permission denied
# 解决方法：chmod +x PoseEvaluator.py
# chmod +x ./PoseEvaluator.py


#rosbag2_1970_01_04-04_45_37
echo "Start to evaluate the oldversion pose..."
    python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_45_37/CameraTrajectory_OF_ICP_Lidar.txt \
    --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_45_37/gt.txt \
    --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_45_37/GEOSLAM.json \
    --t_max_diff 800.0
#rosbag2_1970_01_04-04_36_13
echo "Start to evaluate the oldversion pose..."
    python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_36_13/CameraTrajectory_OF_ICP_Lidar.txt \
    --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_36_13/gt.txt \
    --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_1970_01_04-04_36_13/GEOSLAM.json \
    --t_max_diff 800.0
echo "End to evaluate the oldversion pose..."
#rosbag2_2024_12_24-18_41_50
echo "Start to evaluate the oldversion pose..."
    python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_24-18_41_50/CameraTrajectory_OF_ICP_Lidar.txt \
    --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_24-18_41_50/gt_old.txt \
    --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_24-18_41_50/CameraTrajectory_OF_ICP_Lidar.json \
    --t_max_diff 800.0
echo "End to evaluate the oldversion pose..."
#/home/tingyang/3d_recon/VINS-RGBD_res/vins_result_no_loop.txt
# echo "Start to evaluate the oldversion pose..."
#     python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/VINS-RGBD_res/vins_result_no_loop_seq1.txt \
#     --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/gt_old.txt \
#     --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/VINS.json \
#     --t_max_diff 800.0
# echo "End to evaluate the oldversion pose..."

# echo "Start to evaluate the oldversion pose..."
#     python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/S-VIO_res/svio_no_loop_seq1.csv \
#     --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/gt_old.txt \
#     --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/S-VIO.json \
#     --t_max_diff 800.0
# echo "End to evaluate the oldversion pose..."

# echo "Start to evaluate the oldversion pose..."
#     python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/VINS-Fusion_res/vio.csv \
#     --gt /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/gt_old.txt \
#     --result /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_06-10_35_09/VINS-Fusion.json \
#     --t_max_diff 800.0
# echo "End to evaluate the oldversion pose..."

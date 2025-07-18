import os
import subprocess
import zipfile
import numpy as np
import matplotlib.pyplot as plt
import evo


trajectory_file1 = "/home/tingyang/3d_recon/dataset/G1/rosbag2_2025_07_07-16_31_51/Ours.txt"
ground_truth_file = "/home/users/horizon/g1_map_seq6/gt.txt"
trajectory_file2="/home/tingyang/3d_recon/dataset/G1/rosbag2_2025_07_07-16_31_51/orbslam3.txt"
trajectory_file3 = "/home/tingyang/3d_recon/VINS-RGBD_res/vins_rgbd.txt"
trajectory_file4 = "/home/tingyang/3d_recon/S-VIO_res/svio.csv"
eva_save_path="./"

# rpe_command = f"evo_traj tum {trajectory_file1} {trajectory_file2} {trajectory_file3} --ref={ground_truth_file}  -as -va --plot  --plot_mode xy --t_max_diff 100 --save_plot {eva_save_path}"
# rpe_command = f"evo_ape tum {ground_truth_file} {trajectory_file1} {trajectory_file2} {trajectory_file3}  -as -va --plot  --plot_mode xy --t_max_diff 100 --save_plot {eva_save_path}"


#设置图例名称
rpe_command = f"evo_traj tum {trajectory_file1} {trajectory_file2} {trajectory_file3} {trajectory_file4} --ref={ground_truth_file}  -as -va --plot  --plot_mode xy --t_max_diff 100 --save_plot {eva_save_path}"

os.system(rpe_command)


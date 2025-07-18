# 使用时需更新程序及数据路径
import cv2
import numpy as np
import os
import pdb    
from pprint import pprint
from scipy.spatial.transform import Rotation as R
import multiprocessing
import yaml
from copy import deepcopy
from tqdm import tqdm
import re
# import open3d as o3d
# SH_Fs
# data_root ="/root/horizon/3D_Recon/dataset/L515/record_by_ros/"
# data_root = "/home/users/tingyang.xiao/3D_Recon/datasets/dataset_L515/"
# data_root ="/root/horizon/3D_Recon/dataset/d435i/"
data_root="/root/horizon/navigation/datasets/go2_d435i/"
datalist=[
    "rosbag2_2024_11_13-17_13_31",


]
# 运行orbslam3 
bin_file = "./rgbd_inertial_ros2"
param1 = "../../../Vocabulary/ORBvoc.txt"
param2="./config/d435i.yaml"
command = f"ros2 run {bin_file} {param1} {param2}"
os.system(command)
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
# data_root ="/home/tingyang/3d_recon/dataset/go2_nav/"
# /home/tingyang/3d_recon/dataset/go2_nav/rosbag2_2024_12_24-18_41_50
# data_root = "/home/tingyang/3d_recon/dataset/L515/"
data_root = "/home/tingyang/3d_recon/dataset/go2_nav/"
# data_root ="/root/horizon/navigation/datasets/"
# data_root="/root/horizon/navigation/datasets/go2_d435i/"
# data_root="/mnt/d/3d_recon/dataset/openloris/office/"
# data_root="/root/Dataset/openloris/"
datalist=[

    # "rosbag2_2024_12_06-10_35_09",
    # "rosbag2_2024_12_24-18_41_50",
    # "rosbag2_1970_01_04-04_36_13",
    "rosbag2_1970_01_04-04_45_37"

]




# convert bundle dataset for orbslam3

def generate_association_for_us(color_list, assocaite_path):
    # tuplelist = [("{:.4f}".format(f), "color/"+str(f)+".png", "depth/"+str(f)+".png") for f in color_list]
    color_tuplelist = [(f"{f:.4f}", f"color/{f:.4f}.png",f"depth/{f:.4f}.png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in color_tuplelist:
            file.write(' '.join(record) + "\n")


def remap_colordepth_to_rgbdepth(scan_calib_dict):
    new_scan_calib_dict= deepcopy(scan_calib_dict)
    color_cameraMatrix =  np.array(
                    [[float(scan_calib_dict["rgb.fx"]), 0.0, float(scan_calib_dict["rgb.cx"])],
                       [0.0, float(scan_calib_dict["rgb.fy"]), float(scan_calib_dict["rgb.cy"])],
                       [0.0, 0.0, 1.0]])      

    distCoeffs = np.array([
        float(scan_calib_dict["rgb.k1"]), float(scan_calib_dict["rgb.k2"]), 
        float(scan_calib_dict["rgb.p1"]), float(scan_calib_dict["rgb.p2"]), 
        float(scan_calib_dict["rgb.k3"])])

    # 输出图像的尺寸
    imageSize = (np.int32(scan_calib_dict["depth.width"]), np.int32(scan_calib_dict["depth.height"]))  # 示例图像尺寸
    new_cameraMatrix, roi = cv2.getOptimalNewCameraMatrix(color_cameraMatrix, distCoeffs, imageSize, alpha=0)

    new_scan_calib_dict["rgb.fx"] = new_cameraMatrix[0,0]
    new_scan_calib_dict["rgb.fy"] = new_cameraMatrix[1,1]
    new_scan_calib_dict["rgb.cx"] = new_cameraMatrix[0,2]
    new_scan_calib_dict["rgb.cy"] = new_cameraMatrix[1,2]
    new_scan_calib_dict["rgb.width"] = int(scan_calib_dict["depth.width"])
    new_scan_calib_dict["rgb.height"] = int(scan_calib_dict["depth.height"])
    new_scan_calib_dict["rgb.k1"] = 0
    new_scan_calib_dict["rgb.k2"] = 0
    new_scan_calib_dict["rgb.p1"] = 0
    new_scan_calib_dict["rgb.p2"] = 0
    new_scan_calib_dict["rgb.k3"] = 0

    # 计算映射矩阵
    map1, map2 = cv2.initUndistortRectifyMap(color_cameraMatrix, distCoeffs, np.eye(3), new_cameraMatrix, imageSize, cv2.CV_16SC2)
    img_lst = os.listdir(scan_calib_dict["color_root"])
    depth_list = os.listdir(scan_calib_dict["depth_root"])
    output_rgb_root = scan_calib_dict["rgb_root"]
    output_new_depth_root = scan_calib_dict["new_depth_root"]
    # import pdb; pdb.set_trace() 
    for img in tqdm(img_lst, desc="Processing color images"):
        color_img = cv2.imread(os.path.join(scan_calib_dict["color_root"], img), cv2.IMREAD_COLOR)
        rgb_image = cv2.remap(color_img, map1, map2, interpolation=cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(output_rgb_root, img), rgb_image)

    for img in tqdm(depth_list, desc="Processing depth images"):
        depth_img = cv2.imread(os.path.join(scan_calib_dict["depth_root"], img), cv2.IMREAD_UNCHANGED)
        new_depth_image = cv2.remap(depth_img, map1, map2, interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(output_new_depth_root, img), new_depth_image)
    return new_scan_calib_dict




# bundle_config_file = "/home/wei02feng/rgbd_mapping/dense_mapping/zParameters_us.yaml"
# bundle_params_file = "/home/wei02feng/rgbd_mapping/dense_mapping/zParametersBundlingDefault.yaml"

# default_config={}
# default_config_path = "/home/wei02feng/rgbd_mapping/ORB_SLAM3/Examples/RGB-D/TUM2.yaml"
# with open(default_config_path, 'r') as file:
#     default_config= yaml.safe_load(file)

for data in datalist:
    #读取内参txt
    dataset_path = os.path.join(data_root, data)
    # scan_calib_fpath = os.path.join(dataset_path, "raw_camera_params.yaml")
    # scan_calib_dict = parse_scannet_calib(scan_calib_fpath)

    input_color_root = os.path.join(dataset_path, "color")
    # scan_calib_dict["color_root"] = input_color_root
    input_depth_root = os.path.join(dataset_path, "depth")
    # scan_calib_dict["depth_root"] = input_depth_root

    # output_rgb_root = os.path.join(dataset_path, "color")
    # os.makedirs(output_rgb_root, exist_ok=True)
    # scan_calib_dict["rgb_root"] = output_rgb_root

    # output_depth_root = os.path.join(dataset_path, "depth")
    # os.makedirs(output_depth_root, exist_ok=True)
    # scan_calib_dict["new_depth_root"] = output_depth_root

    # new_scan_calib_dict = remap_colordepth_to_rgbdepth(scan_calib_dict)   

    # #生成associtae.txt(rgb)
    associate_path = os.path.join(dataset_path, "associate.txt") # key/timestamp, for orbslam3
    color_list = os.listdir(input_color_root)
    sorted(color_list)
    depth_list = os.listdir(input_depth_root)
    sorted(depth_list)
    color_frame_list = [np.round(np.float64(re.findall(r'\d+\.\d+', f)[0]), 4) for f in color_list]
    depth_frame_list = [np.round(np.float64(re.findall(r'\d+\.\d+', f)[0]), 4) for f in depth_list]
    color_frame_list.sort()
    depth_frame_list.sort()
    depth_frame_list_diff = [depth_frame_list[i+1] - depth_frame_list[i] for i in range(len(depth_frame_list)-1)]
    color_frame_list_diff = [color_frame_list[i+1] - color_frame_list[i] for i in range(len(color_frame_list)-1)]
    with open(os.path.join(dataset_path, "depth_frame_list_diff.txt"), 'w') as file:
        for record in depth_frame_list_diff:
            file.write(str(record) + "\n")
    with open(os.path.join(dataset_path, "color_frame_list_diff.txt"), 'w') as file:
        for record in color_frame_list_diff:
            file.write(str(record) + "\n")
    frame_list=[]

    frame_list = list(set(color_frame_list).intersection(set(depth_frame_list)))
    frame_list.sort()
    # frame_list相邻求差得到frame_list_diff
    frame_list_diff = [frame_list[i+1] - frame_list[i] for i in range(len(frame_list)-1)]
    # frame_list_diff写入diff.txt
    with open(os.path.join(dataset_path, "diff.txt"), 'w') as file:
        for record in frame_list_diff:
            file.write(str(record) + "\n")
    # generate_association_for_us(frame_list, associate_path)


    # 运行orbslam3 
    # bin_file = "../rgbd_inertial"
    # param1 = "../../../Vocabulary/ORBvoc.txt"
    # param2="../config/L515.yaml"
    # param2="../config/d435i.yaml"
    # param2="../config/openloris.yaml"
    # bin_file = "/home/users/tingyang.xiao/3D_Recon/rgbd_mapping/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial"
    # param1 = "/home/users/tingyang.xiao/3D_Recon/rgbd_mapping/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    # param2="/home/users/tingyang.xiao/3D_Recon/rgbd_mapping/ORB_SLAM3/Examples/RGB-D-Inertial/L515.yaml"
    # param2 = os.path.join(dataset_path, "orbslam3_rgbd.yaml")
    # param3 = dataset_path
    # command1="rm -rf /home/tingyang/3d_recon/rgbd_mapping/ORB_SLAM3/Examples/RGB-D-Inertial/script/logs"
    # command = f"{bin_file} {param1} {param2} {param3} {associate_path}"
    # os.system(command1)
    # os.system(command)
    root_file="/home/tingyang/3d_recon/geoflow-slam/"
    configs=[
        # root_file+"script/run_orbslam/RGBD-Inertial/config/go2_op_icp_lidar_indoor1.yaml",
        # root_file+"script/run_orbslam/RGBD-Inertial/config/go2_op_icp_lidar_indoor2.yaml",
        # root_file+"script/run_orbslam/RGBD-Inertial/config/go2_op_icp_lidar_parking.yaml",
        root_file+"script/run_orbslam/RGBD-Inertial/config/go2_op_icp_lidar_outdoor.yaml",
        ]
    for config in configs:
        print("config: ", config)
        bin_file=root_file+"Examples/RGB-D-Inertial/rgbd_inertial"
        param1 = root_file+"Vocabulary/ORBvoc.txt"
        param2=config
        param3 = dataset_path
        param4 = os.path.join(dataset_path, "associate.txt")
        command = f"{bin_file} {param1} {param2} {param3} {param4}"
        os.system(command)
        # 杀死上一个进程
        os.system("pkill -f rgbd_inertial")
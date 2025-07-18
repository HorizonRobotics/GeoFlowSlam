import cv2
import numpy as np
import os
import pdb    
from pprint import pprint
from scipy.spatial.transform import Rotation as R
import multiprocessing
import yaml


def parse_scannet_calib(example):
    scan_calib_dict = {}
    with open(example, 'r') as file:
        lines = file.readlines()
        for line in lines:
            key, value = line.split('=')
            key = key.strip()
            value = value.strip()
            scan_calib_dict[key] = value
        return scan_calib_dict
    
def parse_scannet_calib_new(scan_calib_new, cali_dir):
    intrinsic_color = np.loadtxt(os.path.join(cali_dir, "intrinsic_color.txt")).reshape((4,4))
    intrinsic_depth = np.loadtxt(os.path.join(cali_dir, "intrinsic_depth.txt")).reshape((4,4))
    extrinsic_color = np.loadtxt(os.path.join(cali_dir, "extrinsic_color.txt")).reshape((4,4))
    extrinsic_depth = np.loadtxt(os.path.join(cali_dir, "extrinsic_depth.txt")).reshape((4,4))
    scan_calib_new["intrinsic_color"] = intrinsic_color
    scan_calib_new["intrinsic_depth"] = intrinsic_depth
    scan_calib_new["extrinsic_color"] = extrinsic_color
    scan_calib_new["extrinsic_depth"] = extrinsic_depth
    return scan_calib_new


def remap_color_to_rgb(scan_calib_dict):
    color_cameraMatrix = np.array(scan_calib_dict["intrinsic_color"]).astype(np.float64)
    new_cameraMatrix = np.array(scan_calib_dict["intrinsic_depth"]).astype(np.float64)
    distCoeffs = np.array([0, 0, 0, 0, 0])
    # 输出图像的尺寸
    imageSize = (np.int32(scan_calib_dict["depthWidth"]), np.int32(scan_calib_dict["depthHeight"]))  # 示例图像尺寸

    # 计算映射矩阵
    map1, map2 = cv2.initUndistortRectifyMap(color_cameraMatrix[:3, :3], distCoeffs, np.eye(3), new_cameraMatrix[:3, :3], imageSize, cv2.CV_16SC2)
    img_lst = os.listdir(scan_calib_dict["color_root"])
    output_rgb_root = scan_calib_dict["rgb_root"]

    for img in img_lst:
        color_img = cv2.imread(os.path.join(scan_calib_dict["color_root"], img), cv2.IMREAD_COLOR)
        rgb_image = cv2.remap(color_img, map1, map2, interpolation=cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(output_rgb_root, img), rgb_image)


def generate_association_for_scannet(color_list, assocaite_path):
    tuplelist = [("{:.6f}".format(f*0.1), "rgb/"+str(f)+".jpg", "depth/"+str(f)+".png") for f in color_list]
    ## for TUM dataset
    # tuplelist = [("{:.6f}".format(f*0.1), "rgb/"+str(f)+".png", "depth/"+str(f)+".png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")
            

def generate_posegt_for_orbslam3_mono(dataset_path, posegt_list, pose_gt_path):
    with open(pose_gt_path, 'w') as file:
        # file.write("#ground truth trajectory \n")
        # file.write("#timestamp tx ty tz qx qy qz qw \n")
        for pose_path in posegt_list:
            key_ts = "{:.6f}".format(pose_path*0.1)
            # print(os.path.join(dataset_path,pose_path))
            value_pose = np.loadtxt(os.path.join(dataset_path, "pose", str(pose_path)+".txt")).reshape((-1, 4))
            if value_pose.shape[0] != 4 or np.isnan(value_pose).any() or np.isinf(value_pose).any():
                continue
            if value_pose[0][0] == float(-1):
                continue
            # 使用scipy.spatial.transform来转换为四元数
            r = R.from_matrix(value_pose[:3, :3])
            quaternion = r.as_quat()  # [x, y, z, w] 的顺序
            position = value_pose[:3, 3]
            line = []
            line.append(key_ts)
            line.extend(position)
            line.extend(quaternion)
            file.write(' '.join(map(str, line)) + "\n")           

# data_root = "/home/users/tingyang.xiao/scannet/original/"
data_root = "/home/tingyang/3d_recon/dataset/scannet/"
# data_root = "/home/tingyang/3d_recon/dataset/mechanical_arm/"
# data_root = "/home/tingyang/3d_recon/dataset/tum_dataset/"
datalist=[
    # "scene0000_00",
    "scene0050_00",
    "scene0059_00",
    "scene0084_00",
    "scene0106_00",
    "scene0169_00",
    "scene0181_00",
    "scene0207_00",
    "scene0580_00",
    "scene0616_00",
    
    # TUM
    # "rgbd_dataset_freiburg1_360",
    # "rgbd_dataset_freiburg1_desk",
    # "rgbd_dataset_freiburg1_desk2",
    # "rgbd_dataset_freiburg1_floor",
    # "rgbd_dataset_freiburg1_plant",
    # "rgbd_dataset_freiburg1_room",
    # "rgbd_dataset_freiburg1_rpy",
    # "rgbd_dataset_freiburg1_xyz",
    # "rgbd_dataset_freiburg1_teddy",
    # "rgbd_dataset_freiburg2_desk",
    # "rgbd_dataset_freiburg2_xyz",


]
for data in datalist:
    dataset_path = os.path.join(data_root, data)
    scan_calib_fpath = os.path.join(dataset_path, data + ".txt")
    scan_calib_dict = parse_scannet_calib(scan_calib_fpath)
    scan_calib_dict = parse_scannet_calib_new(scan_calib_dict, os.path.join(dataset_path, "intrinsic"))
    input_color_root = os.path.join(dataset_path, "color")
    scan_calib_dict["color_root"] = input_color_root
    output_rgb_root = os.path.join(dataset_path, "rgb")
    os.makedirs(output_rgb_root, exist_ok=True)
    scan_calib_dict["rgb_root"] = output_rgb_root
    # pprint(scan_calib_dict)
# 
    # remap_color_to_rgb(scan_calib_dict)
    pose_gt_path = os.path.join(dataset_path, "groundtruth.txt") # key/timestamp, camera to world, tx,ty,tz, qx,qy,qz,qw, for evaluation;
    associate_path = os.path.join(dataset_path, "associate.txt") # key/timestamp, for orbslam3
    posegt_list = os.listdir(os.path.join(dataset_path, "pose"))
    color_list = os.listdir(output_rgb_root)
    # color_list = os.listdir(input_color_root)
    frame_list = [np.int32(f.split('.')[0]) for f in color_list]
    frame_list.sort()
    generate_association_for_scannet(frame_list, associate_path)
    # generate_posegt_for_orbslam3_mono(dataset_path, frame_list, pose_gt_path)


    # bin_file = "/home/users/tingyang.xiao/3D_Recon/rgbd_mapping/ORB_SLAM3/Examples/RGB-D/rgbd_tum"
    root_file="/home/tingyang/3d_recon/rgbd_mapping/ORB_SLAM3"
    configs=[
        # root_file+"/Examples/RGB-D/config/ScanNet/scannet_icp_op.yaml",
            #  root_file+"/Examples/RGB-D/config/ScanNet/scannet_icp_no_op.yaml",
            #  root_file+"/Examples/RGB-D/config/ScanNet/scannet_no_icp_op.yaml",
            # root_file+"/Examples/RGB-D/config/ScanNet/scannet_lidar.yaml",
            root_file+"/Examples/RGB-D/config/ScanNet/scannet_icp_op_lidar.yaml",
             ]
    for config in configs:
        bin_file=root_file+"/Examples/RGB-D/rgbd_tum"
        # param1 = "/home/users/tingyang.xiao/3D_Recon/rgbd_mapping/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        param1 = root_file+"/Vocabulary/ORBvoc.txt"
        param2=config
        param3 = dataset_path
        param4 = os.path.join(dataset_path, "associate.txt")
        command = f"{bin_file} {param1} {param2} {param3} {param4}"
        os.system(command)
        # 杀死上一个进程
        os.system("pkill -f rgbd_tum")
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
    # tuplelist = [("{:.6f}".format(f*0.1), "rgb/"+str(f)+".jpg", "depth/"+str(f)+".png") for f in color_list]
    ## for TUM dataset
    tuplelist = [("{:.6f}".format(f*0.1), "rgb/"+str(f)+".png", "depth/"+str(f)+".png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")
            
def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    # first_keys = first_list.keys()
    # second_keys = second_list.keys()
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    # pdb.set_trace()
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            # matches.append((diff, first_list[a], second_list[b]))
            # print(str(a))
            line = str(a) + ' ' + first_list[a][0] + ' ' + second_list[b][0]
            matches.append(line)
            first_keys.remove(a)
            second_keys.remove(b)
            
    
    matches.sort()
    return matches
def read_file_list(filename,remove_bounds=False):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    if remove_bounds:
        lines = lines[100:-100]
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    # list = [(f"{np.float64(l[0]):.4f}", l[1:]) for l in list if len(l) > 1]
    list = [(np.float64(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)
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

data_root = "/home/tingyang/3d_recon/dataset/tum_dataset/"
datalist=[
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
    # fr3 Structure vs. Texture
    # "rgbd_dataset_freiburg3_long_office_household",
    ## challenging
    # "rgbd_dataset_freiburg3_nostructure_notexture_far",
    "rgbd_dataset_freiburg3_nostructure_notexture_near_withloop",
    
    # "rgbd_dataset_freiburg3_nostructure_texture_far",
    # "rgbd_dataset_freiburg3_nostructure_texture_near_withloop",
    # "rgbd_dataset_freiburg3_structure_notexture_far",
    # "rgbd_dataset_freiburg3_structure_notexture_near",
    # "rgbd_dataset_freiburg3_structure_notexture_far",
    # "rgbd_dataset_freiburg3_structure_texture_far",
    # "rgbd_dataset_freiburg3_structure_texture_near",
    # "rgbd_dataset_freiburg3_large_cabinet",

]
for data in datalist:
    dataset_path = os.path.join(data_root, data)
    # print(dataset_path)


    pose_gt_path = os.path.join(dataset_path, "groundtruth.txt") # key/timestamp, camera to world, tx,ty,tz, qx,qy,qz,qw, for evaluation;
    associate_path = os.path.join(dataset_path, "associate.txt") # key/timestamp, for orbslam3

    color_list = read_file_list(os.path.join(dataset_path, "rgb.txt"))
    # 取出color_list第一列时间戳
    color_frame_list=[key for key in color_list]
    #转float
    color_frame_list = [np.float64(i) for i in color_frame_list]
    # print(color_list)
    # print(color_frame_list)
    depth_list = read_file_list(os.path.join(dataset_path, "depth.txt"))
    # 取出depth_list第一列时间戳
    depth_frame_list=[key for key in depth_list]
    #转float
    depth_frame_list = [np.float64(i) for i in depth_frame_list]
    # sorted(depth_list)
    # print(depth_list)

    matches = associate(color_list, depth_list,0.0,0.0333) 
    result_file = os.path.join(dataset_path, 'associate.txt')
    # depth_frame_list_diff = [depth_frame_list[i+1] - depth_frame_list[i] for i in range(len(depth_frame_list)-1)]
    # color_frame_list_diff = [color_frame_list[i+1] - color_frame_list[i] for i in range(len(color_frame_list)-1)]
    # with open(os.path.join(dataset_path, "depth_frame_list_diff.txt"), 'w') as file:
    #     for record in depth_frame_list_diff:
    #         file.write(str(record) + "\n")
    # with open(os.path.join(dataset_path, "color_frame_list_diff.txt"), 'w') as file:
    #     for record in color_frame_list_diff:
    #         file.write(str(record) + "\n")
    with open(result_file, 'w') as file:
        for line in matches:
            line = line + "\n"
            # 保留四位小数
            # print(f"{np.float64(line[0]):.4f}")
            # print(line[0])
            # 安照空格分割取出第一个元素，转为浮点数，保留四位小数
            # file.write(f"{np.float64(line.split(' ')[0]):.4f} {line.split(' ')[1]} {line.split(' ')[2]}")
            file.write(f"{np.float64(line.split(' ')[0])} {line.split(' ')[1]} {line.split(' ')[2]}")

    root_file="/home/tingyang/3d_recon/rgbd_mapping/ORB_SLAM3"
    configs=[
        # root_file+"/Examples/RGB-D/config/TUM/tum1/tum_icp_no_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum1/tum_no_icp_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum1/tum_icp_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum1/tum_only_lidar.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum1/tum_icp_op_lidar.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum3/tum_icp_no_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum3/tum_no_icp_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum3/tum_icp_op.yaml",
        # root_file+"/Examples/RGB-D/config/TUM/tum3/tum_only_lidar.yaml",
        root_file+"/Examples/RGB-D/config/TUM/tum3/tum_icp_op_lidar.yaml",
        ]
    for config in configs:
        bin_file=root_file+"/Examples/RGB-D/rgbd_tum"
        param1 = root_file+"/Vocabulary/ORBvoc.txt"
        param2=config
        param3 = dataset_path
        param4 = os.path.join(dataset_path, "associate.txt")
        command = f"{bin_file} {param1} {param2} {param3} {param4}"
        os.system(command)
        # 杀死上一个进程
        # os.system("pkill -f rgbd_tum")
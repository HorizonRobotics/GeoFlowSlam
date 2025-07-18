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
data_root = "/home/tingyang/3d_recon/dataset/L515/"
data_root = "/home/tingyang/3d_recon/dataset/go2_nav/"
# data_root ="/root/horizon/navigation/datasets/"
# data_root="/root/horizon/navigation/datasets/go2_d435i/"
# data_root="/mnt/d/3d_recon/dataset/openloris/office/"
# data_root="/root/Dataset/openloris/"
datalist=[
    # "2024-10-11-14-54-39",
    # "2024-10-11-14-58-17",
    # "2024-10-16-16-18-48",
    "2024-10-16-16-21-57",
    # "2024-10-16-16-27-20",
    # "2024-10-16-16-29-36",
    # "2024-10-16-16-33-20",
    # "rosbag2_2024_11_13-17_13_31",
    # "2024-11-15-21-07-36",
    # "2024-11-15-21-13-27"
    # "2024-11-18-16-09-08",
    # "2024-12-09-17-19-36",
    # "2024-12-09-18-55-00",
    # "2024-12-09-18-57-04"
    # "2024-12-11-18-48-42",
    # "2024-12-12-15-08-01",
    # "2024-12-23-13-23-23",
    # "rosbag2_2024-12-23-13-23-23",
    "rosbag2_2024_12_06-10_35_09"
    # "rosbag2_2024_12_24-18_41_50",
    # "cafe/cafe1-1",
    # "cafe/cafe1-2",
    # "office/office1-1",
    # "office/office1-2",
    # "office/office1-3",
    # "office/office1-4",
    # "office/office1-5",
    # "office/office1-6",
    # "office/office1-7",

]


def parse_scannet_calib(example):
    scan_calib_dict = {}
    with open(example, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():
                key, value = line.split(': ')
                key = key.strip()
                value = value.strip()
                scan_calib_dict[key] = value
        return scan_calib_dict

def parse_scannet_calib2(example):
    scan_calib_dict = {}
    with open(example, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():
                key, value = line.split('=')
                key = key.strip()
                value = value.strip()
                scan_calib_dict[key] = value
        return scan_calib_dict

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

# def run_rgbd_integration_save(rgbd_dict, intrinsic, mesh_file):
#     dataset_path = os.path.dirname(mesh_file)
#     # 视频输出文件名
#     output_filename = os.path.join(dataset_path, 'sensor_view.mp4')
#     # 编码器选择（根据你的需求选择适当的编码器）
#     fourcc = cv2.VideoWriter_fourcc('M', 'P', '4', 'V')  # 也可以使用 'MJPG', 'MP4V', 'X264' 等
#     # 帧率
#     fps = 10
#     # 视频分辨率（必须与输入图像的分辨率相同）
#     frame_width = 640*2
#     frame_height = 480
#     # frame_width = 848*2
#     # frame_height = 480
#     import open3d as o3d

#     # 创建 VideoWriter 对象
#     out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
#     volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         voxel_length=8.0 / 512.0,
#         sdf_trunc=0.1,
#         color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

#     for i in range(len(rgbd_dict["camera_poses"])):
#         print("Integrate {:d}-th image into the volume.".format(i))
#         if i not in rgbd_dict["color_paths"].keys():
#             continue
#         color = o3d.io.read_image(rgbd_dict["color_paths"][i])
#         depth = o3d.io.read_image(rgbd_dict["depth_paths"][i])

#         color_image = cv2.imread(rgbd_dict["color_paths"][i], cv2.IMREAD_COLOR)
#         depth_image = cv2.imread(rgbd_dict["depth_paths"][i], cv2.IMREAD_UNCHANGED)

#         # 将深度图像转换为伪彩色图像
#         depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         # 将彩色图像和深度图像拼接在一起
#         combined_image = np.hstack((color_image, depth_color_map))
#         out.write(combined_image)

#         # out.release()
#         rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color, depth, depth_scale= 1000.0 , depth_trunc=3.0, convert_rgb_to_intensity=False)
#         volume.integrate(
#                 rgbd,
#                 intrinsic,
#                 np.linalg.inv(rgbd_dict["camera_poses"][i]))
#     print("Extract a triangle mesh from the volume and visualize it.")
#     mesh = volume.extract_triangle_mesh()

#     # 应用 Laplacian 平滑
#     # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
#     # 应用网格简化
#     # mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=50000)
#     # 移除孤立的顶点和面
#     mesh.remove_duplicated_vertices()
#     mesh.remove_degenerate_triangles()
#     mesh.remove_duplicated_triangles()
#     # 应用网格简化
#     mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=500000)
#     # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)

#     mesh.compute_vertex_normals()

#     # mesh_file = os.path.join(os.path.dirname(rgbd_dict["color_paths"][0]), "open3d_mesh.ply")
#     print("save mesh to : ", mesh_file)
#     o3d.io.write_triangle_mesh(mesh_file, mesh)
#     # o3d.visualization.draw_geometries([mesh],
#     #                                   front=[0.5297, -0.1873, -0.8272],
#     #                                   lookat=[2.0712, 2.0312, 1.7251],
#     #                                   up=[-0.0558, -0.9809, 0.1864],
#     #                                   zoom=0.47)

def quaternion_to_rotation_matrix(q):
    # 提取四元式的各个分量
    x, y, z, w = q

    # 计算旋转矩阵
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

    return R
def create_transformation_matrix(R, T):
    """
    将旋转矩阵 R 和位移向量 T 拼接成一个 4x4 变换矩阵。

    参数:
    R -- 3x3 旋转矩阵
    T -- 3x1 位移向量

    返回:
    4x4 变换矩阵
    """
    # 确保 R 是 3x3 矩阵
    assert R.shape == (3, 3), "旋转矩阵 R 必须是 3x3 矩阵"
    # 确保 T 是 3x1 向量
    assert T.shape == (3, 1), "位移向量 T 必须是 3x1 向量"

    # 创建一个 4x4 的单位矩阵
    transformation_matrix = np.eye(4)

    # 填充旋转矩阵 R
    transformation_matrix[:3, :3] = R

    # 填充位移向量 T
    transformation_matrix[:3, 3] = T.flatten()

    return transformation_matrix

def read_orbslam_pose(slam_pose_path):
    pose_dict = {}
    with open(slam_pose_path, 'r') as file:
        lines = file.readlines()
    for line in lines:
        line_vec  = line.split(' ')
        key_ts = line_vec[0]
        position = np.array([line_vec[1:4]]).astype(np.float64) #cam to world
        quaternion = np.array(line_vec[4:8]).astype(np.float64)
        rotation_matrix = quaternion_to_rotation_matrix(quaternion)
        pose = create_transformation_matrix(rotation_matrix, position.T)
        pose_dict[key_ts] = pose
    return pose_dict

def read_associatoin_rgbd_dict(association_path):
    rgbd_dict = dict()
    with open(association_path, 'r') as file:
        lines = file.readlines()
    for line in lines:
        line_vec  = line.split(' ')
        key_ts = line_vec[0].strip()
        rgbd_tuple = (line_vec[1].strip(), line_vec[2].strip())
        rgbd_dict[key_ts] = rgbd_tuple
    return rgbd_dict



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
    root_file="/home/tingyang/3d_recon/rgbd_mapping/ORB_SLAM3/"
    configs=[
        # root_file+"Examples/RGB-D-Inertial/config/op.yaml",
        # root_file+"Examples/RGB-D-Inertial/config/icp.yaml",
        # root_file+"Examples/RGB-D-Inertial/config/only_lidar.yaml",
        # root_file+"Examples/RGB-D-Inertial/config/op_icp.yaml",
        # root_file+"Examples/RGB-D-Inertial/config/op_icp_lidar.yaml",
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
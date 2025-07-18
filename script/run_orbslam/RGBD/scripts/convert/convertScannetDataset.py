import cv2
import numpy as np
import os
import pdb    
from pprint import pprint
from scipy.spatial.transform import Rotation as R
import multiprocessing
import yaml
import json
# from Examples.Calibration.python_scripts.process_imu import dataset

# from Examples.Calibration.python_scripts.process_imu import dataset

# convert rgbd into orbslam style
# convert pose-gt into orbslam sytle
# convert calib into orbslam style

data_root = "/mnt/disk2/scans_test/"

bundle_root = "/home/zxl/robotics/dev/perception/dataset/bundle_fusion_testdata/"

orb_root = "/home/zxl/robotics/dev/perception/dataset/"

handhele_root = "/home/zxl/realsense/handheld/"

go2_root = "/home/zxl/realsense/go2/"

orb_datalist = [
    "rgbd_dataset_freiburg3_long_office_household",
    "rgbd_dataset_freiburg3_nostructure_texture_far", # 强纹理，弱结构，远距离
    "rgbd_dataset_freiburg3_nostructure_notexture_far", # 弱纹理，弱结构，远距离
    "rgbd_dataset_freiburg3_structure_texture_far", # 强纹理，强结构，远距离
    "rgbd_dataset_freiburg3_structure_texture_near", # 强纹理，强结构，近距离
    "rgbd_dataset_freiburg3_structure_notexture_far", # 弱纹理，强结构，远距离
    "rgbd_dataset_freiburg3_structure_notexture_near", # 弱纹理，强结构，近距离
    "rgbd_dataset_freiburg3_nostructure_notexture_near_withloop_validation", # 挑战数据
]

handheld_datalist = {
    # "IC10_Room",
    # "IC10_coffeeroom",
    # "IC10_Desktop",
    "IC10_Corridorr",
    # "IC10_corridorr_loop",
}

go2_datalist = [

]

bundle_datalist=[
    # "copyroom",
    # "apt0",
    # "apt1",
    # "apt2",
    "office0",
    "office1",
    "office2",
    "office3",
]

datalist=[
    "scene0707_00",
    "scene0708_00",
    "scene0709_00",
    "scene0710_00",
    "scene0711_00",
    "scene0712_00",
    "scene0713_00",
    "scene0714_00",
    "scene0715_00",
    "scene0716_00",
    "scene0717_00",
    "scene0718_00",
    "scene0719_00",
    "scene0720_00",
    "scene0721_00",
    "scene0722_00",
    "scene0723_00",
    "scene0724_00",
    "scene0725_00",
    "scene0726_00",
    "scene0727_00",
    "scene0728_00",
    "scene0729_00",
    "scene0730_00",
    "scene0731_00",
    "scene0732_00",
    "scene0733_00",
    "scene0734_00",
    "scene0735_00",
    "scene0736_00",
    "scene0737_00",
    "scene0738_00",
]


def parse_scannet_calib(example):
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

# convert calib into infinitam style
infinitam_calib_fname = "infiniTAM_calib_new_new.txt"

def convertCalibToInfinTAM(scan_dict, dataset_path):
    example_path = os.path.join(dataset_path, infinitam_calib_fname)
    with open(example_path, 'w') as file:
        # 写入单行文本
        lines = []
        line = scan_dict["colorWidth"] + " " + scan_dict["colorHeight"] + "\n"
        lines.append(line)
        line = scan_dict["fx_color"] + " " + scan_dict["fy_color"] + "\n"
        lines.append(line)
        line = scan_dict["mx_color"] + " " + scan_dict["my_color"] + "\n"
        lines.append(line)
        lines.append("\n")


        # depth intrinsic
        line = scan_dict["depthWidth"] + " " + scan_dict["depthHeight"] + "\n"
        lines.append(line)
        line = scan_dict["fx_depth"] + " " + scan_dict["fy_depth"] + "\n"
        lines.append(line)
        line = scan_dict["mx_depth"] + " " + scan_dict["my_depth"] + "\n"
        lines.append(line)
        lines.append("\n")

        # color to depth extrinsic
        extrinsic = str(scan_dict["colorToDepthExtrinsics"]).split(' ')        
        line = ' '.join(extrinsic[:4]) + "\n"
        lines.append(line)
        line = ' '.join(extrinsic[4:8]) + "\n"
        lines.append(line)
        line = ' '.join(extrinsic[8:12]) + "\n"
        lines.append(line)
        lines.append("\n")        

        # depth scale info
        line = "0 0\n"
        lines.append(line)

        # 写入多行文本
        file.writelines(lines)

def convertCalibToInfinTAMNew(scan_dict, dataset_path):
    example_path = os.path.join(dataset_path, infinitam_calib_fname)
    with open(example_path, 'w') as file:
        # 写入单行文本
        lines = []
        # color intrinsic
        # line = scan_dict["colorWidth"] + " " + scan_dict["colorHeight"] + "\n"
        # lines.append(line)
        # line = str(scan_dict["intrinsic_color"][0, 0]) + " " + str(scan_dict["intrinsic_color"][1, 1]) + "\n"
        # lines.append(line)
        # line = str(scan_dict["intrinsic_color"][0, 2]) + " " + str(scan_dict["intrinsic_color"][1, 2]) + "\n"
        # lines.append(line)
        # lines.append("\n")
        line = scan_dict["depthWidth"] + " " + scan_dict["depthHeight"] + "\n"
        lines.append(line)
        line = str(scan_dict["intrinsic_depth"][0, 0]) + " " + str(scan_dict["intrinsic_depth"][1, 1]) + "\n"
        lines.append(line)
        line = str(scan_dict["intrinsic_depth"][0, 2]) + " " + str(scan_dict["intrinsic_depth"][1, 2]) + "\n"
        lines.append(line)
        lines.append("\n")

        # depth intrinsic
        line = scan_dict["depthWidth"] + " " + scan_dict["depthHeight"] + "\n"
        lines.append(line)
        line = str(scan_dict["intrinsic_depth"][0, 0]) + " " + str(scan_dict["intrinsic_depth"][1, 1]) + "\n"
        lines.append(line)
        line = str(scan_dict["intrinsic_depth"][0, 2]) + " " + str(scan_dict["intrinsic_depth"][1, 2]) + "\n"
        lines.append(line)
        lines.append("\n")

        # rgbcamera to world
        # depth to world
        # color to depth
        Twc = scan_dict["extrinsic_color"]
        Twd = scan_dict["extrinsic_depth"]
        Tdc = np.linalg.inv(Twd) @ Twc

        # color to depth extrinsic
        line = ' '.join(map(str, Tdc[0, :])) + "\n"
        lines.append(line)
        line = ' '.join(map(str, Tdc[1, :])) + "\n"
        lines.append(line)
        line = ' '.join(map(str, Tdc[2, :])) + "\n"
        lines.append(line)
        lines.append("\n")        

        # depth scale info
        line = "0 0\n"
        lines.append(line)

        # 写入多行文本
        file.writelines(lines)


# # process scannet dataset for infinitam
# for data in datalist:
#     dataset_path = os.path.join(data_root, data)
#     scan_calib_fpath = os.path.join(dataset_path, data+".txt")
#     scan_calib_dict = parse_scannet_calib(scan_calib_fpath)
#     scan_calib_dict = parse_scannet_calib_new(scan_calib_dict, os.path.join(dataset_path, "intrinsic"))
#     pprint(scan_calib_dict)
#     convertCalibToInfinTAMNew(scan_calib_dict, dataset_path)

# convert bundle dataset for orbslam3
  # 
def generate_association_for_orbslam3(color_list, depth_list, assocaite_path):
    tuplelist = [("{:.6f}".format(np.int32(f.split('.')[0].split('-')[-1])*0.1), f, f.replace("color.jpg", "depth.png")) for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")

def generate_association_for_bundle(color_list, assocaite_path):
    tuplelist = [("{:.6f}".format(f*0.1), "color/"+str(f)+".jpg", "depth/"+str(f)+".png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")


def generate_association_for_scannet(color_list, assocaite_path):
    tuplelist = [("{:.6f}".format(f*0.1), "rgb/"+"{:06d}".format(f)+".jpg", "depth/"+str(f)+".png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")


def generate_association_for_handheld(color_list, assocaite_path):
    tuplelist = [("{:.6f}".format(f*0.1), "color/"+"{:06d}".format(f)+".jpg", "depth/"+"{:06d}".format(f)+".png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")

def generate_association_for_bundlefusion(pose_list, assocaite_path):
    tuplelist = [("{:.6f}".format(np.int32(f.split('.')[0].split('-')[-1])*0.1), f.replace("pose.txt", "color.jpg"), f.replace("pose.txt", "depth.png")) for f in pose_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")


def generate_association_for_orbslam3_mono(color_list, assocaite_path):
    tuplelist = [("{:.6f}".format(f*0.1), "{:.6f}".format(f)+".jpg") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in tuplelist:
            file.write(' '.join(record) + "\n")


def generate_posegt_for_orbslam3(dataset_path, posegt_list, pose_gt_path):
    with open(pose_gt_path, 'w') as file:
        # file.write("#ground truth trajectory \n")
        # file.write("#timestamp tx ty tz qx qy qz qw \n")
        for pose_path in posegt_list:
            key_ts = "{:.6f}".format(np.int32(pose_path.split('.')[0].split('-')[-1])*0.1)
            # print(os.path.join(dataset_path,pose_path))
            value_pose = np.loadtxt(os.path.join(dataset_path,pose_path)).reshape((-1, 4))
            if value_pose.shape[0] != 4 or value_pose.shape[1] != 4 or value_pose[0][0] is float(-1):
                print(pose_path)
                print("value_pose: ", value_pose)
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


def generate_posegt_for_orbslam3_mono(dataset_path, posegt_list, pose_gt_path):
    with open(pose_gt_path, 'w') as file:
        # file.write("#ground truth trajectory \n")
        # file.write("#timestamp tx ty tz qx qy qz qw \n")
        for pose_path in posegt_list:
            key_ts = "{:.6f}".format(pose_path*0.1)
            # print(os.path.join(dataset_path,pose_path))
            value_pose = np.loadtxt(os.path.join(dataset_path, "pose", str(pose_path)+".txt")).reshape((-1, 4))
            if value_pose.shape[0] != 4:
                continue
            if value_pose[0][0] is float(-1):
                continue
            if abs(value_pose[0][0]) > 1:
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

def generate_posegt_for_orbslam3_mono_origin_firstframe(dataset_path, posegt_list, pose_gt_path):
    with open(pose_gt_path, 'w') as file:
        # file.write("#ground truth trajectory \n")
        # file.write("#timestamp tx ty tz qx qy qz qw \n")

        firstframe_valuepose = np.loadtxt(os.path.join(dataset_path, "pose", str(posegt_list[0])+".txt")).reshape((-1, 4))
        r = R.from_matrix(firstframe_valuepose[:3, :3])
        quaternion = r.as_quat()  # [x, y, z, w] 的顺序
        position = firstframe_valuepose[:3, 3].reshape((1,3))
        T_firstframe_to_world = create_transformation_matrix(r.as_matrix()[:3, :3], position.T)
        T_world_to_firstframe = np.linalg.inv(T_firstframe_to_world)

        for pose_path in posegt_list:
            key_ts = "{:.6f}".format(pose_path*0.1)
            # print(os.path.join(dataset_path,pose_path))
            value_pose = np.loadtxt(os.path.join(dataset_path, "pose", str(pose_path)+".txt")).reshape((-1, 4))
            if value_pose.shape[0] != 4:
                continue
            if value_pose[0][0] is float(-1):
                continue
            if abs(value_pose[0][0]) > 1:
                continue
            # 使用scipy.spatial.transform来转换为四元数
            r = R.from_matrix(value_pose[:3, :3])
            quaternion = r.as_quat()  # [x, y, z, w] 的顺序
            position = value_pose[:3, 3].reshape((1,3))
            T_cam_to_world = create_transformation_matrix(r.as_matrix()[:3, :3], position.T)
            T_cam_to_firstframe = T_world_to_firstframe @ T_cam_to_world

            r = R.from_matrix(T_cam_to_firstframe[:3, :3])
            position = T_cam_to_firstframe[:3, 3]
            quaternion = r.as_quat()
            line = []
            line.append(key_ts)
            line.extend(position)
            line.extend(quaternion)
            file.write(' '.join(map(str, line)) + "\n")

def find_files_with_extension(directory, extension):
    # List all files and directories in the specified directory
    all_files = os.listdir(directory)
    # Filter files that end with the specified extension
    matching_files = [f for f in all_files if f.endswith(extension)]
    return matching_files


def run_parse(command):
    print(f"Executing: {command}")
    os.system(command)


# # process bundle fusion for all scannet dataset
# for data in datalist:
#     bin_file = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/cmake-build-release/bundle_fusion_example"
#     param1 = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/zParametersScanNet.yaml"
#     param2 = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/zParametersBundlingDefault.yaml"
#     param3 = os.path.join("/mnt/disk2/scans_test/scene0707_00/", data, "associate.txt")
#     command = f"{bin_file} {param1} {param2} {param3}"
#     run_parse(command)



# process scannet
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
        id = int(img.split('.')[0])
        # pdb.set_trace()
        color_img = cv2.imread(os.path.join(scan_calib_dict["color_root"], img), cv2.IMREAD_COLOR)
        rgb_image = cv2.remap(color_img, map1, map2, interpolation=cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(output_rgb_root, str("%06d" % id) + ".jpg"), rgb_image)

# from ruamel.yaml import YAML
# yaml_old = YAML()
# def write_yaml(file_path, data):
#     with open(file_path, 'w') as file:
#         yaml_old.dump(data, file)


import open3d as o3d
from PIL import Image
def run_rgbd_integration(rgbd_dict, intrinsic):
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=4.0 / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    for i in range(len(rgbd_dict["camera_poses"])):
    # for i in range(2000):
        print("Integrate {:d}-th image into the volume.".format(i))
        if i not in rgbd_dict["color_paths"].keys():
            continue
        color = o3d.io.read_image(rgbd_dict["color_paths"][i])
        depth = o3d.io.read_image(rgbd_dict["depth_paths"][i])
        # pdb.set_trace()

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1000.0, depth_trunc=4.0, convert_rgb_to_intensity=False)
        volume.integrate(
            rgbd,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
            np.linalg.inv(rgbd_dict["camera_poses"][i]))
    print("Extract a triangle mesh from the volume and visualize it.")
    mesh = volume.extract_triangle_mesh()
    # 应用 Laplacian 平滑
    # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    # 移除孤立的顶点和面
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    # 应用网格简化
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=500000)
    # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    mesh.compute_vertex_normals()
    mesh_file = os.path.join(os.path.dirname(rgbd_dict["color_paths"][0]), "infinitam_mesh_orbpose_keyframe.ply")
    print("save mesh to : ", mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh)
    # o3d.visualization.draw_geometries([mesh],
    #                               front=[0.5297, -0.1873, -0.8272],
    #                               lookat=[2.0712, 2.0312, 1.7251],
    #                               up=[-0.0558, -0.9809, 0.1864],
    #                               zoom=0.47)
    # exit(1)


# #
# def checkLoopingSuccess(slam_pose_path):
#     with open(slam_pose_path, 'r') as file:
#         lines = file.readlines()
#     first_line_qw = lines[0].split(' ')[-1]
#     for i in range(1, len(lines)):
#         if lines[i].split(' ')[-1] == first_line_qw:
#             return False
#     return True
#
#
# slam_succsss = dict()
# for data in bundle_datalist:
#     dataset_path = os.path.join(bundle_root, data)
#     slam_pose_path = os.path.join(dataset_path, "CameraTrajectory.txt")
#     slam_succsss[data] = checkLoopingSuccess(slam_pose_path)
#
# pprint(slam_succsss)


def run_rgbd_integration_save(rgbd_dict, intrinsic, mesh_file):
    dataset_path = os.path.dirname(mesh_file)
    # 视频输出文件名
    output_filename = os.path.join(dataset_path, 'sensor_view.mp4')
    # 编码器选择（根据你的需求选择适当的编码器）
    fourcc = cv2.VideoWriter_fourcc(*'X264')  # 也可以使用 'MJPG', 'MP4V', 'X264' 等
    # 帧率
    fps = 30
    # 视频分辨率（必须与输入图像的分辨率相同）
    frame_width = 640*2
    frame_height = 480

    # 创建 VideoWriter 对象
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=8.0 / 512.0,
        sdf_trunc=0.1,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    for i in range(len(rgbd_dict["camera_poses"])):
    # for i in range(20):
        print("Integrate {:d}-th image into the volume.".format(i))
        if i not in rgbd_dict["color_paths"].keys():
            continue
        color = o3d.io.read_image(rgbd_dict["color_paths"][i])
        depth = o3d.io.read_image(rgbd_dict["depth_paths"][i])

        color_image = cv2.imread(rgbd_dict["color_paths"][i], cv2.IMREAD_COLOR)
        depth_image = cv2.imread(rgbd_dict["depth_paths"][i], cv2.IMREAD_UNCHANGED)

        # 将深度图像转换为伪彩色图像
        depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # 将彩色图像和深度图像拼接在一起
        combined_image = np.hstack((color_image, depth_color_map))
        out.write(combined_image)

        out.release()
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False)
        volume.integrate(
            rgbd,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
            np.linalg.inv(rgbd_dict["camera_poses"][i]))
    print("Extract a triangle mesh from the volume and visualize it.")
    mesh = volume.extract_triangle_mesh()

    # 应用 Laplacian 平滑
    # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    # 应用网格简化
    # mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=50000)
    # 移除孤立的顶点和面
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    # 应用网格简化
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=500000)
    # mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)


    mesh.compute_vertex_normals()

    # mesh_file = os.path.join(os.path.dirname(rgbd_dict["color_paths"][0]), "open3d_mesh.ply")
    print("save mesh to : ", mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh)
    # o3d.visualization.draw_geometries([mesh],
    #                                   front=[0.5297, -0.1873, -0.8272],
    #                                   lookat=[2.0712, 2.0312, 1.7251],
    #                                   up=[-0.0558, -0.9809, 0.1864],
    #                                   zoom=0.47)


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
def read_tum_pose(slam_pose_path):
    pose_dict = {}
    with open(slam_pose_path, 'r') as file:
        lines = file.readlines()
    for line in lines:
        line_vec  = line.strip().split(' ')
        key_ts = line_vec[0]
        position = np.array([line_vec[1:4]]).astype(np.float64) #cam to world
        quaternion = np.array(line_vec[4:8]).astype(np.float64)
        rotation_matrix = quaternion_to_rotation_matrix(quaternion)
        pose = create_transformation_matrix(rotation_matrix, position.T)
        pose_dict[key_ts] = pose # camerea to world
    return pose_dict

def align_to_gtworld_pose_transform(orb_pose_dict, first_to_world, pose_aligned_path):
    pose_aligned = dict()
    with open(pose_aligned_path, 'w') as file:
        # file.write("#ground truth trajectory \n")
        # file.write("#timestamp tx ty tz qx qy qz qw \n")
        for key_ts, campose_to_first in orb_pose_dict.items():
            pose_align = first_to_world @ campose_to_first
            pose_aligned[key_ts] = pose_align
            # 使用scipy.spatial.transform来转换为四元数
            r = R.from_matrix(pose_align[:3, :3])
            quaternion = r.as_quat()  # [x, y, z, w] 的顺序
            position = pose_align[:3, 3]
            line = []
            line.append(key_ts)
            line.extend(position)
            line.extend(quaternion)
            file.write(' '.join(map(str, line)) + "\n")

    return pose_aligned

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



default_config_path = "/home/zxl/robotics/dev/perception/ORB_SLAM3_OLD/Examples/RGB-D/TUM2.yaml"
with open(default_config_path, 'r') as file:
    default_config= yaml.safe_load(file)



bundle_config_file = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/zParametersScanNet.yaml"
bundle_params_file = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/zParametersBundlingDefault.yaml"

#
# for data in handheld_datalist:
#     dataset_path = os.path.join(handhele_root, data)
#     associate_path = os.path.join(dataset_path, "associate.txt") # key/timestamp, for orbslam3
#     color_path = os.path.join(dataset_path, "color")
#     color_list = os.listdir(color_path)
#     frame_list = [np.int32(f.split('.')[0]) for f in color_list]
#     frame_list.sort()
#     generate_association_for_handheld(frame_list, associate_path)
#
#     # generate config file for orbslam3-binfile
#     scannet_yaml_file = os.path.join(dataset_path, "scannet.yaml")
#     rgbd_realsense_handheld_yaml_file = os.path.join(dataset_path, "rgbd_realsense_handheld.yaml")
#     with open(rgbd_realsense_handheld_yaml_file, 'r') as file:
#         rgbd_config= yaml.safe_load(file)
#
#
#     bundle_config_dict = parse_scannet_calib(bundle_config_file)
#     bundle_config_dict["s_colorWidth"] = rgbd_config["Camera.width"]
#     bundle_config_dict["s_colorHeight"] = rgbd_config["Camera.height"]
#     bundle_config_dict["s_depthWidth"] = rgbd_config["Camera.width"]
#     bundle_config_dict["s_depthHeight"] = rgbd_config["Camera.height"]
#
#     bundle_config_dict["s_cameraIntrinsicFx"] = str(rgbd_config["Camera1.fx"])
#     bundle_config_dict["s_cameraIntrinsicFy"] = str(rgbd_config["Camera1.fy"])
#     bundle_config_dict["s_cameraIntrinsicCx"] = str(rgbd_config["Camera1.cx"])
#     bundle_config_dict["s_cameraIntrinsicCx"] = str(rgbd_config["Camera1.cy"])
#
#     bundle_config_dict["d_cameraIntrinsicFx"] = str(rgbd_config["Camera1.fx"])
#     bundle_config_dict["d_cameraIntrinsicFy"] = str(rgbd_config["Camera1.fy"])
#     bundle_config_dict["d_cameraIntrinsicCx"] = str(rgbd_config["Camera1.cx"])
#     bundle_config_dict["d_cameraIntrinsicCx"] = str(rgbd_config["Camera1.cy"])
#
#     #update and write yaml
#     default_config["Camera1.fx"] = rgbd_config["Camera1.fx"]
#     default_config["Camera1.fy"] = rgbd_config["Camera1.fy"]
#     default_config["Camera1.cx"] = rgbd_config["Camera1.cx"]
#     default_config["Camera1.cy"] = rgbd_config["Camera1.cy"]
#     default_config["Camera.width"] = rgbd_config["Camera.width"]
#     default_config["Camera.height"] = rgbd_config["Camera.height"]
#     default_config["RGBD.DepthMapFactor"] = 1000.0
#
#     with open(scannet_yaml_file, 'w') as file:
#         yaml.dump(default_config, file)
#
#     with open(scannet_yaml_file, 'r') as file:
#         current_content = file.read()
#     # new_content = current_content
#     new_content = "%YAML:1.0\n" + current_content
#     with open(scannet_yaml_file, 'w') as file:
#         file.write(new_content)
#
#     # 写入文件
#     bundle_scannet_file = os.path.join(dataset_path, "bundle.yaml")
#     with open(bundle_scannet_file, 'w') as file:
#         for key, value in bundle_config_dict.items():
#             file.write(f'{key} = {value}\n')
#
#     # pdb.set_trace()
#
#     # # # run orbslam3 pose/reloc program
#     bin_file = "/home/zxl/robotics/dev/perception/ORB_SLAM3/Examples/RGB-D/rgbd_tum"
#     param1 = "/home/zxl/robotics/dev/perception/ORB_SLAM3/Vocabulary/ORBvoc.txt"
#     param2 = os.path.join(dataset_path, "scannet.yaml")
#     param3 = dataset_path
#     param4 = os.path.join(dataset_path, "associate.txt")
#     command = f"{bin_file} {param1} {param2} {param3} {param4}"
#     run_parse(command)
#
#     pdb.set_trace()
#
#     # # run bundle fusion
#     bin_file = "/home/zxl/robotics/dev/perception/BundleFusion_Ubuntu_Pangolin/cmake-build-release/bundle_fusion_example"
#     param1 = bundle_scannet_file
#     param2 = bundle_params_file
#     param3 = associate_path
#     command = f"{bin_file} {param1} {param2} {param3} "
#     run_parse(command)
#
#
#
#
# pdb.set_trace()
# exit(1)

# compute odometry scale for tum format file.write("#timestamp tx ty tz qx qy qz qw \n")
def compute_odometry_scale(ref_odo_path, est_odo_path):
    gt_pose_dict = read_tum_pose(ref_odo_path)
    est_pose_dict = read_tum_pose(est_odo_path)
    common_keys = gt_pose_dict.keys() & est_pose_dict.keys()
    orb_pose_sum_distance = 0
    gt_pose_sum_distance = 0
    id = 0
    for key_ts in common_keys:
        if id == 0:
            Twc_prev_orb = est_pose_dict[key_ts].copy()
            Twc_prev_gt = gt_pose_dict[key_ts].copy()
        else:
            Twc_orb = est_pose_dict[key_ts].copy()
            Twc_gt = gt_pose_dict[key_ts].copy()
            Tpc_orb = np.linalg.inv(Twc_prev_orb) @ Twc_orb # curr to prev transformation
            Tpc_gt = np.linalg.inv(Twc_prev_gt) @ Twc_gt
            orb_pose_sum_distance += np.linalg.norm(Tpc_orb[:3, 3])
            gt_pose_sum_distance += np.linalg.norm(Tpc_gt[:3, 3])
            Twc_prev_orb = Twc_orb
            Twc_prev_gt = Twc_gt
        id = id + 1



    scale = gt_pose_sum_distance / orb_pose_sum_distance  * 1.0
    pose_num = len(common_keys)
    return scale, pose_num


for data in datalist:
    dataset_scale_json = os.path.join(data_root, data, "scale_info_newversion.json")
    dataset_scale_dict = dict()
    dataset_scale_dict["dataset_name"] = data
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

    bundle_config_dict = parse_scannet_calib(bundle_config_file)


    output_rgb_root = os.path.join(dataset_path, "rgb")
    os.makedirs(output_rgb_root, exist_ok=True)

    remap_color_to_rgb(scan_calib_dict)
    # generate rgb/depth association for orbslam3-binfile
    # generate pose-gt for pose evaluation
    pose_gt_path = os.path.join(dataset_path, "groundtruth.txt") # key/timestamp, camera to world, tx,ty,tz, qx,qy,qz,qw, for evaluation;
    pose_gt_path_origin_firstframe = os.path.join(dataset_path, "groundtruth_origin_firstframe.txt") # key/timestamp, camera to world, tx,ty,tz, qx,qy,qz,qw, for evaluation;

    associate_path = os.path.join(dataset_path, "associate.txt") # key/timestamp, for orbslam3
    posegt_list = os.listdir(os.path.join(dataset_path, "pose"))
    color_list = os.listdir(output_rgb_root)
    frame_list = [np.int32(f.split('.')[0]) for f in color_list]
    frame_list.sort()
    generate_association_for_scannet(frame_list, associate_path)
    generate_posegt_for_orbslam3_mono(dataset_path, frame_list, pose_gt_path)
    generate_posegt_for_orbslam3_mono_origin_firstframe(dataset_path, frame_list, pose_gt_path_origin_firstframe)

    bundle_config_dict["s_colorWidth"] = scan_calib_dict["depthWidth"]
    bundle_config_dict["s_colorHeight"] = scan_calib_dict["depthHeight"]
    bundle_config_dict["s_depthWidth"] = scan_calib_dict["depthWidth"]
    bundle_config_dict["s_depthHeight"] = scan_calib_dict["depthHeight"]

    bundle_config_dict["s_cameraIntrinsicFx"] = str(scan_calib_dict["intrinsic_depth"][0, 0])
    bundle_config_dict["s_cameraIntrinsicFy"] = str(scan_calib_dict["intrinsic_depth"][1, 1])
    bundle_config_dict["s_cameraIntrinsicCx"] = str(scan_calib_dict["intrinsic_depth"][0, 2])
    bundle_config_dict["s_cameraIntrinsicCx"] = str(scan_calib_dict["intrinsic_depth"][1, 2])

    bundle_config_dict["d_cameraIntrinsicFx"] = str(scan_calib_dict["intrinsic_depth"][0, 0])
    bundle_config_dict["d_cameraIntrinsicFy"] = str(scan_calib_dict["intrinsic_depth"][1, 1])
    bundle_config_dict["d_cameraIntrinsicCx"] = str(scan_calib_dict["intrinsic_depth"][0, 2])
    bundle_config_dict["d_cameraIntrinsicCx"] = str(scan_calib_dict["intrinsic_depth"][1, 2])

    # 写入文件
    bundle_scannet_file = os.path.join(dataset_path, "bundle.yaml")
    with open(bundle_scannet_file, 'w') as file:
        for key, value in bundle_config_dict.items():
            file.write(f'{key} = {value}\n')

    #update and write yaml
    default_config["Camera1.fx"] = float(scan_calib_dict["intrinsic_depth"][0][0])
    default_config["Camera1.fy"] = float(scan_calib_dict["intrinsic_depth"][1][1])
    default_config["Camera1.cx"] = float(scan_calib_dict["intrinsic_depth"][0][2])
    default_config["Camera1.cy"] = float(scan_calib_dict["intrinsic_depth"][1][2])
    default_config["Camera.width"] = int(scan_calib_dict["depthWidth"])
    default_config["Camera.height"] = int(scan_calib_dict["depthHeight"])
    default_config["RGBD.DepthMapFactor"] = 1000.0
    default_config["Camera1.k1"] = 0.0
    default_config["Camera1.k2"] = 0.0
    default_config["Camera1.p1"] = 0.0
    default_config["Camera1.p2"] = 0.0
    default_config["Camera1.k3"] = 0.0

    # generate config file for orbslam3-binfile
    scannet_yaml_file = os.path.join(dataset_path, "scannet.yaml")
    # write_yaml(scannet_yaml_file, default_config)
    with open(scannet_yaml_file, 'w') as file:
        yaml.dump(default_config, file)

    with open(scannet_yaml_file, 'r') as file:
        current_content = file.read()

    # new_content = "#"+current_content
    new_content = "%YAML:1.0\n" + current_content

    with open(scannet_yaml_file, 'w') as file:
        file.write(new_content)



    # # # run orbslam3 pose/reloc program
    bin_file = "/home/zxl/robotics/dev/perception/ORB_SLAM3/Examples/RGB-D/rgbd_tum"
    param1 = "/home/zxl/robotics/dev/perception/ORB_SLAM3_OLD/Vocabulary/ORBvoc.txt"
    param2 = os.path.join(dataset_path, "scannet.yaml")
    param3 = dataset_path
    param4 = os.path.join(dataset_path, "associate.txt")
    command = f"{bin_file} {param1} {param2} {param3} {param4}"
    run_parse(command)


    # evo all frame pose
    param1 = os.path.join(dataset_path, "groundtruth.txt")
    param2 = os.path.join(dataset_path, "CameraTrajectory_newversion.txt")
    param3 = os.path.join(dataset_path, "evo_pose_orb_allframe_newversion.pdf")
    if os.path.exists(param3):
        command = f"rm -rf {param3}"
        run_parse(command)
    command = f"conda run -n nerfstudio evo_traj tum --ref {param1} {param2} --t_max_diff 0.02 --sync -as -v --save_plot {param3}"
    run_parse(command)
    scale, pose_num = compute_odometry_scale(param1, param2)
    print("all frame scale correction, pose_num : ", scale, pose_num)
    dataset_scale_dict["all_frame_scale"] = 1.0 / scale
    dataset_scale_dict["all_frame_num"] = pose_num

    # evo all keyframe pose
    param1 = os.path.join(dataset_path, "groundtruth.txt")
    param2 = os.path.join(dataset_path, "KeyFrameTrajectory_newversion.txt")
    param3 = os.path.join(dataset_path, "evo_pose_orb_keyframe_newversion.pdf")
    if os.path.exists(param3):
        command = f"rm -rf {param3}"
        run_parse(command)
    command = f"conda run -n nerfstudio evo_traj tum --ref {param1} {param2} --t_max_diff 0.02 --sync -as -v --save_plot {param3} "
    run_parse(command)
    scale, pose_num = compute_odometry_scale(param1, param2)
    scale, pose_num = compute_odometry_scale(param1, param2)
    print("keyframe frame scale correction, pose_num : ", scale, pose_num)
    dataset_scale_dict["keyframe_scale"] = 1.0 / scale
    dataset_scale_dict["keyframe_num"] = pose_num

    with open(dataset_scale_json, "w") as json_file:
        json.dump(dataset_scale_dict, json_file, indent=4)  # `indent` 用于格式化输出，4 表示缩进级别



pdb.set_trace()
exit(1)



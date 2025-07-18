# Copyright 2020 <Copyright hobot>
# @date 22/Dec./2020
# @author Jiaxin Zhang

import os
import json
from warnings import warn
from re import match
from collections import defaultdict
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from PoseTransformer import invT


def get_packdirs(dataset_root, checked=False):
    """get all packdirs in absolute paths

    Args:
        dataset_root (string): absolute path to dataset dir, e.g. /bucket/output/J5FSD/FSD_Urban_v3
        checked (bool, optional):
            If set to true, return packs that are in checklist with newest checked dates.
            If set to false, iterate through whole datasets (with regex) and return all available packs. Defaults to False.

    Returns:
        list: list of absolute paths of each pack, can be used as input of DatasetReader.read_pack()
    """
    if checked:
        return __get_checked_packdirs(dataset_root)
    else:
        return __get_all_packdirs(dataset_root)


def __get_all_packdirs(dataset_root):
    """get all packdirs in absolute paths, iterate through whole datasets (with regex) and return all available packs

    Args:
        dataset_root (string): absolute path to dataset dir, e.g. /bucket/output/J5FSD/FSD_Urban_v3

    Returns:
        list: list of absolute paths of each pack, can be used as input of DatasetReader.read_pack()
    """
    packdir_paths = []
    for date_dir in os.listdir(dataset_root):
        if match("^[a-zA-Z0-9]*_\d\d\d\d\d\d\d\d_D$", date_dir):  # noqa
            root_date_dir = os.path.join(dataset_root, date_dir)
            for pack_dir in os.listdir(root_date_dir):
                if match("^\d\d\d\d\d\d\d\d-\d\d\d\d\d\d_\d\d\d$", pack_dir):  # noqa
                    root_date_pack_dir = os.path.join(root_date_dir, pack_dir)
                    packdir_paths.append(root_date_pack_dir)
    return packdir_paths


def __get_checked_packdirs(dataset_root):
    """return packs that are in checklist with newest checked dates

    Args:
        dataset_root (string): absolute path to dataset dir, e.g. /bucket/output/J5FSD/FSD_Urban_v3

    Raises:
        FileNotFoundError: if checklist not exists
    Returns:
        list: list of absolute paths of each pack, can be used as input of DatasetReader.read_pack()
    """
    checklist_dir = os.path.join(dataset_root, "checklist")
    checklists = os.listdir(checklist_dir)
    valid_checklists = []
    for checklist in checklists:
        if match("^\d\d\d\d\d\d\d\d.txt$", checklist):  # noqa
            valid_checklists.append(checklist)
    if len(valid_checklists) == 0:
        raise FileNotFoundError("can not find valid checklists, try to use get_packdirs(root, checked=False)")
    valid_checklists.sort(reverse=True)
    packdir_paths = read_checklist(dataset_root, valid_checklists[0])
    return packdir_paths


def read_checklist(dataset_root, checklist):
    """read checklist from dataset

    Args:
        dataset_root (string): absolute path to dataset dir, e.g. /bucket/output/J5FSD/FSD_Urban_v3
        checklist (string): checklist file name, e.g. 20210703.txt

    Raises:
        FileNotFoundError: if checklist file not found

    Returns:
        list: list of pack dir in absolute path
    """
    checklist_dir = os.path.join(dataset_root, "checklist")
    if not os.path.isdir(checklist_dir):
        raise FileNotFoundError("{} does not exist, try to use get_packdirs(root, checked=False)".format(checklist_dir))
    file_path = os.path.join(checklist_dir, checklist)
    with open(file_path, 'r') as file:
        packdir_list = file.readlines()
    # [:-1] is to remove \n at the end of each line
    packdir_paths = [os.path.join(dataset_root, packdir[:-1]) for packdir in packdir_list]
    return packdir_paths


class DatasetReader:
    def __init__(self, pack_path=None):
        self.attribute = dict()
        self.extrinsic_graph = None
        self.pack_path = ""
        if pack_path:
            self.read_pack(pack_path)

    def read_pack(self, path):
        """use this function to load pack meta data before using other functions

        Args:
            path (string): absolute path to dataset dir,
                           e.g. /dataset/J5FSD/FSD_Urban_v4/DG202_20210620_D

        Raises:
            FileNotFoundError: if failed to find attribute.json under path
        """
        self.pack_path = path

        # read attribute
        attribute_path = os.path.join(self.pack_path, "attribute.json")
        if os.path.exists(attribute_path):
            with open(attribute_path, 'r') as file:
                self.attribute = json.load(file)
        else:
            raise FileNotFoundError("can not find {}".format(attribute_path))

    def get_sensor_timestamp(self, sensor_name, sync=True):
        sync_string = "sync" if sync else "unsync"
        timestamps = np.asarray(self.attribute[sync_string][sensor_name]).astype(np.double)
        timestamps /= 1000.0
        return timestamps.tolist()

    def yield_lidar(self, start_idx=None, end_idx=None, deskewed=False):
        for filename in self.yield_sensor_filepath("lidar_top", "bin", start_idx=start_idx, end_idx=end_idx):
            # x y z intensity ring timestamps
            filename = filename.replace("lidar_top", "deskewed_lidar_top") if deskewed else filename
            pointcloud = np.fromfile(filename, dtype=np.double).reshape(-1, 6)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, pointcloud

    def yield_deskewed_lidar(self, start_idx=8, end_idx=-8):
        for filename in self.yield_sensor_filepath("lidar_top", "bin", start_idx=start_idx, end_idx=end_idx):
            filename = filename.replace("lidar_top", "deskewed_lidar_top")
            # x y z intensity ring timestamps
            pointcloud = np.fromfile(filename, dtype=np.double).reshape(-1, 6)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, pointcloud

    def yield_painted_lidar(self, start_idx=8, end_idx=-8):
        for filename in self.yield_sensor_filepath("lidar_top", "bin", start_idx=start_idx, end_idx=end_idx):
            filename = filename.replace("lidar_top", "painted_lidar_top")
            # x y z intensity ring timestamps semantic_label
            pointcloud = np.fromfile(filename, dtype=np.double).reshape(-1, 7)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, pointcloud

    def yield_IMU_old(self, start_idx=None, end_idx=None,
                      start_time=None, end_time=None):
        for filename in self.yield_sensor_filepath("IMU", "json", sync=False,
                                                   start_idx=start_idx, end_idx=end_idx,
                                                   start_time=start_time, end_time=end_time):
            with open(filename, 'r') as file:
                IMU_frame = json.load(file)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, IMU_frame

    def yield_IMU(self, start_idx=None, end_idx=None,
                  start_time=None, end_time=None):
        data_json_path = os.path.join(self.pack_path, "IMU", "data.json")
        if not os.path.exists(data_json_path):
            warn("Warning(Deprecation): using yield_IMU_old\
                  automatically because IMU/data.json not found")
            yield from self.yield_IMU_old(start_idx=start_idx,
                                          end_idx=end_idx,
                                          start_time=start_time,
                                          end_time=end_time)
            return
        with open(data_json_path, 'r') as file:
            data_json = json.load(file)
        for timestamp in self.attribute["unsync"]["IMU"]:
            IMU_frame = data_json[str(timestamp)]
            timestamp_sec = float(timestamp) / 1000.0
            yield timestamp_sec, IMU_frame

    def yield_GNSS_old(self, start_idx=None, end_idx=None):
        for filename in self.yield_sensor_filepath("UB482", "json", sync=False, start_idx=start_idx, end_idx=end_idx):
            with open(filename, 'r') as file:
                GNSS_frame = json.load(file)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, GNSS_frame

    def yield_GNSS(self, start_idx=None, end_idx=None):
        data_json_path = os.path.join(self.pack_path, "UB482", "data.json")
        if not os.path.exists(data_json_path):
            warn("Warning(Deprecation): using yield_GNSS_old \
                 automatically because UB482/data.json not found")
            yield from self.yield_GNSS_old(start_idx=start_idx,
                                           end_idx=end_idx)
            return
        with open(data_json_path, 'r') as file:
            data_json = json.load(file)
        for timestamp in self.attribute["unsync"]["UB482"]:
            GNSS_frame = data_json[str(timestamp)]
            timestamp_sec = float(timestamp) / 1000.0
            yield timestamp_sec, GNSS_frame

    def yield_camera(self, camera="camera_front", start_idx=None, end_idx=None):
        for filename in self.yield_sensor_filepath(camera, "jpg", start_idx=start_idx, end_idx=end_idx):
            image = cv2.imread(filename)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, image

    def yield_semantic(self, camera="camera_front", start_idx=None, end_idx=None):
        for filename in self.yield_sensor_filepath(camera, "png", start_idx=start_idx, end_idx=end_idx):
            filename = filename.replace(camera, "seg_" + camera)
            image = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, image

    def yield_wheel_old(self, sync=True, start_idx=None, end_idx=None):
        for filename in self.yield_sensor_filepath("wheel", "json", sync=sync,
                                                   start_idx=start_idx,
                                                   end_idx=end_idx):
            with open(filename, 'r') as file:
                wheel_dict = json.load(file)
            timestamp = self.__filepath2timestamp(filename)
            yield timestamp, wheel_dict

    def yield_wheel(self, sync=True, start_idx=None, end_idx=None):
        data_json_path = os.path.join(self.pack_path, "wheel", "data.json")
        if not os.path.exists(data_json_path):
            warn("Warning(Deprecation): using yield_wheel_old \
                 automatically because wheel/data.json not found")
            yield from self.yield_wheel_old(sync=sync, start_idx=start_idx,
                                            end_idx=end_idx)
            return
        with open(data_json_path, 'r') as file:
            data_json = json.load(file)
        for timestamp in self.attribute["sync" if sync else "unsync"]["wheel"]:
            wheel_dict = data_json[str(timestamp)]
            timestamp_sec = float(timestamp) / 1000.0
            yield timestamp_sec, wheel_dict

    @ staticmethod
    def __filepath2timestamp(filepath):
        base_name = os.path.basename(filepath)
        prefix = base_name.split('.')[0]
        timestamp = float(prefix)/1000.0
        return timestamp

    def yield_sensor_filepath(self, sensor_name, ext, sync=True,
                              start_idx=None, end_idx=None,
                              start_time=None, end_time=None):
        """yield absoltue sensor file path of a sensor
        Note that (start_idx, end_idx) and (start_time, end_time) can not be used in the same time.
        If all of these arguments are None, then all frames will be yield.
        Use list(yield_sensor_filepath) if you want a list instead of a generator.
        Note: not suitable for IMU, GNSS, wheel, \
            these sensors are written into single json file.

        Args:
            sensor_name (string): e.g. camera_front, lidar_top
            ext (extension): e.g. jpg, png, bin
            sync (bool, optional): Pick sensor timestamp from sync or unsync(raw data). Defaults to True.
            start_idx (int, optional): start idx to specify. Defaults to None.
            end_idx (int, optional): end idx to specify. Defaults to None.
            start_time (float, optional): start time to specify. Defaults to None.
            end_time (float, optional): end time to specify. Defaults to None.

        Yields:
            string: absolute path to a frame of senor file
        """
        sensor_group = "sync" if sync else "unsync"
        sensor_list = self.attribute[sensor_group][sensor_name]
        sensor_timestamps = np.asarray(sensor_list)/1000.
        sensor_dir = os.path.join(self.pack_path, sensor_name)
        if start_time is None and end_time is None:
            sensor_list = sensor_list[start_idx:end_idx]
        else:
            if start_time is None or start_time <= sensor_timestamps[0]:
                start_idx = None
            elif start_time > sensor_timestamps[-1]:
                start_idx = -1
            else:
                start_idx = np.searchsorted(sensor_timestamps, start_time, side="left")
            if end_time is None or end_time >= sensor_timestamps[-1]:
                end_idx = None
            elif end_idx < sensor_timestamps[0]:
                end_idx = -1
            else:
                end_idx = np.searchsorted(sensor_timestamps, end_time, side="left")-1
            if start_idx < 0 or end_idx < 0:
                sensor_list = []
            else:
                sensor_list = sensor_list[start_idx:end_idx]
        for sensor_ts in sensor_list:
            sensor_file = os.path.join(sensor_dir, "{}.{}".format(sensor_ts, ext))
            yield sensor_file

    def __get_extrinsic(self, from_sensor, to_sensor):
        # 0. return Identity
        if from_sensor == to_sensor:
            return np.eye(4, dtype=np.float32)

        name = "{}_2_{}".format(from_sensor, to_sensor)

        # 1. if name exist done
        if name in self.attribute["calibration"]:
            return np.asarray(self.attribute["calibration"][name])

        # 2. if inverse name exist
        inverse_name = "{}_2_{}".format(to_sensor, from_sensor)
        if inverse_name in self.attribute["calibration"]:
            transform = np.asarray(self.attribute["calibration"][inverse_name])
            return invT(transform)

        # 3. can not get extrinsic directly
        return None

    def __build_extrinsic_graph(self):
        graph = defaultdict(list)
        # Loop to iterate over every edge of the graph
        for sensor_pair in self.attribute["calibration"]:
            if "_2_" in sensor_pair:  # get all extrinsic
                sensor_a, sensor_b = sensor_pair.split('_2_')
                # Creating the graph as adjacency list
                graph[sensor_a].append(sensor_b)
                graph[sensor_b].append(sensor_a)
        self.extrinsic_graph = graph

    def get_extrinsic_path(self, from_sensor, to_sensor):
        """find shortest path between two sensors
            used for self.get_extrinsic()

        Args:
            from_sensor (string): sensor_names, e.g. camera_front, lidar_top, chassis, IMU
            to_sensor (string): sensor_names, e.g. camera_front, lidar_top, chassis, IMU
        """
        if self.extrinsic_graph is None:
            self.__build_extrinsic_graph()
        explored = []
        # Queue for traversing the
        # graph in the BFS
        queue = [[from_sensor]]

        # If the desired node is reached
        if from_sensor == to_sensor:
            return None

        # Loop to traverse the graph
        # with the help of the queue
        while queue:
            path = queue.pop(0)
            node = path[-1]
            # Condition to check if the
            # current node is not visited
            if node not in explored:
                neighbours = self.extrinsic_graph[node]
                # Loop to iterate over the
                # neighbours of the node
                for neighbour in neighbours:
                    new_path = list(path)
                    new_path.append(neighbour)
                    queue.append(new_path)
                    # Condition to check if the
                    # neighbour node is the goal
                    if neighbour == to_sensor:
                        return new_path
                explored.append(node)
        return None

    def get_extrinsic(self, from_sensor, to_sensor):
        """get extrinsic transform matrix between ANY two sensors, can be interprated by:
        The coordinate of a point in {from_sensor} can be transformed into {to_sensor}

        Note that the function use shortest path algorithm to find connect path between
        from_sensor and to_sensor. Thus you can get extrinsic between ANY sensors.

        Args:
            from_sensor (string): sensor_names, e.g. camera_front, lidar_top, chassis, IMU
            to_sensor (string): sensor_name, e.g. camera_front, lidar_top, chassis, IMU

        Returns:
            ndarray: a 4x4 transform matrix in ndarray
        """
        extrinsic_naive_attemp = self.__get_extrinsic(from_sensor, to_sensor)
        if extrinsic_naive_attemp is not None:
            return extrinsic_naive_attemp

        extrinsic = np.eye(4, dtype=np.float32)
        extrinsic_path = self.get_extrinsic_path(from_sensor, to_sensor)
        if extrinsic_path is None:
            print("extrinsic path not found!")
            return None
        else:
            for i in range(len(extrinsic_path)-1):
                extrinsic = self.__get_extrinsic(extrinsic_path[i], extrinsic_path[i+1]) @ extrinsic
            return extrinsic

    def get_all_sensors(self):
        """get list of all available sensors from the dataset

        Returns:
            list: list of all sensors' name
        """
        sensor_list = []
        for sensor_pair in self.attribute["calibration"]:
            sensor_list += sensor_pair.split('_2_')
        return list(set(sensor_list))

    def get_intrinsic(self, sensor):
        """get intrinsic of a sensor

        Args:
            sensor (string): camera name, e.g. camera_front, camera_rear_left

        Returns:
            tuple of ndarray:
                camera_intrinsic_matrix: 3x3 ndarray
                camera_distortion: 1d ndarray, length could be 4, 5, 8 or more,
                                   following opencv convention
        """
        warn("get_intrinsic() is deprecated, use get_intrinsics() instead")
        K = np.asarray(self.attribute["calibration"][sensor]["K"])
        d = np.asarray(self.attribute["calibration"][sensor]["d"])
        return K, d

    def get_intrinsics(self, sensor):
        """get intrinsics of a sensor (K, d, width, height, fov, etc.)

        Args:
            sensor (string): camera name, e.g. camera_front, camera_rear_left

        Returns:
            dict: intrinsics of the sensor
        """
        sensor_intrinsic = self.attribute["calibration"][sensor]
        intrinsics = dict()
        intrinsics["K"] = np.asarray(sensor_intrinsic.get("K", None))
        intrinsics["d"] = np.asarray(sensor_intrinsic.get("d", None))
        intrinsics["width"] = sensor_intrinsic.get("image_width", None)
        intrinsics["height"] = sensor_intrinsic.get("image_height", None)
        intrinsics["hfov"] = sensor_intrinsic.get("fov", None)
        return intrinsics

    def get_GNSS_tum(self):
        """get GNSS/GPS poses in tum array

        Returns:
            ndarray: (N, 7): timestamp(secs), x, y, z, qx, qy, qz, qw
        """
        tum_list = []

        # check gnss json version, use corresponding decoder
        for time, gnss_json in self.yield_GNSS():
            if "x" in gnss_json["position"]:
                gnss_2_tum = self.__gnss_json_2_tum_line_v2
            else:
                gnss_2_tum = self.__gnss_json_2_tum_line_v1
            break
        for time, gnss_json in self.yield_GNSS():
            line = gnss_2_tum(time, gnss_json)
            tum_list.append(line)
        tum_array = np.asarray(tum_list)
        return tum_array

    def __gnss_json_2_tum_line_v1(self, time, gnss_json):
        """read gnss json frame and collect info for tum format:
           timestamp x y z qx qy qz qw
           v1 is for pypackstreamer result

        Args:
            time (double): unix timestamp in seconds
            wheel_json (dict): wheel odometry json frame

        Returns:
            list of double: [timestamp x y z qx qy qz qw]
        """
        warn("Warning(Deprecation): clip/pack results extracted by packstreamer will not be supported in the future")
        line = [time, gnss_json["position"][0], gnss_json["position"][1], gnss_json["position"][2], gnss_json["orientation"]
                [0], gnss_json["orientation"][1], gnss_json["orientation"][2], gnss_json["orientation"][3]]
        return line

    def __gnss_json_2_tum_line_v2(self, time, gnss_json):
        """read gnss json frame and collect info for tum format:
           timestamp x y z qx qy qz qw
           v2 is for tat result

        Args:
            time (double): unix timestamp in seconds
            wheel_json (dict): wheel odometry json frame

        Returns:
            list of double: [timestamp x y z qx qy qz qw]
        """
        line = [time, gnss_json["position"]["x"], gnss_json["position"]["y"], gnss_json["position"]["z"], gnss_json["orientation"]
                ["x"], gnss_json["orientation"]["y"], gnss_json["orientation"]["z"], gnss_json["orientation"]["w"]]
        return line

    def get_wheel_tum(self, sync=False):
        """get wheel(CAN) poses in tum array
        Args:
            sync (bool, optional): whether to read json file from synced messages. Defaults to False.
            Usually, sync messages is synced with LiDAR, thus 10HZ, and unsync is 30HZ.

        Returns:
            ndarray: (N, 7): timestamp(secs), x, y, z, qx, qy, qz, qw
        """
        tum_list = []

        # check wheel json version, use corresponding decoder
        for time, wheel_json in self.yield_wheel(sync=sync):
            if "roll" in wheel_json:
                wheel_2_tum = self.__wheel_json_2_tum_line_v1
            else:
                wheel_2_tum = self.__wheel_json_2_tum_line_v2
            break
        for time, wheel_json in self.yield_wheel(sync=sync):
            line = wheel_2_tum(time, wheel_json)
            tum_list.append(line)
        tum_array = np.asarray(tum_list)
        return tum_array

    def __wheel_json_2_tum_line_v1(self, time, wheel_json):
        """read wheel odometry json frame and collect info for tum format:
           timestamp x y z qx qy qz qw
           v1 is for pypackstreamer result

        Args:
            time (double): unix timestamp in seconds
            wheel_json (dict): wheel odometry json frame

        Returns:
            list of double: [timestamp x y z qx qy qz qw]
        """
        warn("Warning(Deprecation): clip/pack results extracted by packstreamer will not be supported in the future")
        euler_rpy = [wheel_json["roll"], wheel_json["pitch"], wheel_json["yaw"]]
        quaternion = R.from_euler("XYZ", euler_rpy, degrees=False).as_quat()
        line = [time, wheel_json["x"], wheel_json["y"], wheel_json["z"], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        return line

    def __wheel_json_2_tum_line_v2(self, time, wheel_json):
        """read wheel odometry json frame and collect info for tum format:
           timestamp x y z qx qy qz qw
           v2 is for tat result

        Args:
            time (double): unix timestamp in seconds
            wheel_json (dict): wheel odometry json frame

        Returns:
            list of double: [timestamp x y z qx qy qz qw]
        """
        euler_rpy = [0, 0, wheel_json["yaw"]]
        quaternion = R.from_euler("XYZ", euler_rpy, degrees=False).as_quat()
        line = [time, wheel_json["x"], wheel_json["y"], 0, quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        return line

    def get_odometry(self, name_txt):
        odometry_path = os.path.join(self.pack_path, "odometry", name_txt)
        return np.loadtxt(odometry_path)

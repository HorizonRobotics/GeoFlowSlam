# Copyright 2023 <Copyright hobot>
# @date 15/Feb./2023
# @author Jiaxin Zhang


import os
from horizon_driving_dataset.DatasetReader import DatasetReader
from horizon_driving_dataset.RawPackReader import ImagePackReader, PointcloudPackReader  # noqa: E501


def get_pack_root_robust(plate_date_dir):
    """get raw pack root dir in absolute path

    Args:
        plate_date_dir (string): example: "LS912_20221119_D"

    Raises:
        FileNotFoundError: can not find raw pack root in heuristic search

    Returns:
        string: absolute path to raw pack root
    """
    input_bucket_root = ["/horizon-bucket/auto_data",
                         "/horizon-bucket/auto_data_2",
                         "/horizon-bucket/auto_data_4"]
    input_bucket_subdir = ["adas_data_raw/pack/white_changan", "adas-data-upload"]  # noqa E501
    for bucket_root in input_bucket_root:
        for bucket_sub in input_bucket_subdir:
            input_root = os.path.join(bucket_root, bucket_sub, plate_date_dir)
            if os.path.isdir(input_root):
                return input_root
    raise FileNotFoundError("can not find source pack files!")


class DatasetPackReader(DatasetReader):
    def __init__(self, pack_path):
        super().__init__(pack_path)
        self.raw_pack_readers = dict()
        self.raw_pack_suffix = {
            "lidar_top": 30,
            "camera_front_right": 0,
            "camera_rear_right": 1,
            "camera_front_left": 2,
            "camera_rear_left": 3,
            "camera_rear": 4,
            "camera_front": 5,
            "camera_front_30fov": 10
        }

    def update_pack_suffix(self, suffix_dict):
        """Update default pack suffix, for example
        if ADAS_xxx_5.pack is the pack for camera_front, then
        suffix_dict = {"camera_front": 5}

        Args:
            suffix_dict (dict): the input suffix_dict will overwrite the default suffix_dict  # noqa E501
        """
        self.raw_pack_suffix.update(suffix_dict)

    def get_plate_date_dir(self):
        pack_path = str(self.pack_path)
        clip_name = pack_path.split("/")[-1]
        plate = clip_name.split("_")[0]
        date = clip_name.split("_")[1]
        plate_date_dir = "{}_{}_D".format(plate, date)
        return plate_date_dir

    def get_pack_ids(self):
        pack_path = str(self.pack_path)
        site_name = pack_path.split("/")[-2]
        if "Site_" in site_name:  # a clip in a site
            pack_ids = []
            # a clip may contain images from multiple packs
            for key in self.attribute.keys():
                # check self.attribute[key] is a dict
                if not isinstance(self.attribute[key], dict):
                    continue
                pack_id = self.attribute[key].get("pack", None)
                if pack_id is not None:
                    pack_ids.append(pack_id)
            return pack_ids
        else:  # single pack as a clip
            return pack_path.split("/")[-1]

    def get_raw_pack_paths(self, sensor_name, prefix="ADAS"):
        plate_date_dir = self.get_plate_date_dir()
        raw_pack_root = get_pack_root_robust(plate_date_dir)
        pack_ids = self.get_pack_ids()
        suffix = self.raw_pack_suffix[sensor_name]
        pack_paths = []
        for pack_id in pack_ids:
            pack_path = os.path.join(raw_pack_root, f"{prefix}_{pack_id}_{suffix}.pack")  # noqa E501
            pack_paths.append(pack_path)
        return pack_paths

    def get_camera_by_timestamp(self, camera_name, timestamp):
        """get camera image in ndarray by timestamp

        Args:
            camera_name (string): camera name, e.g. "camera_front"
            timestamp (int): timestamp in milliseconds

        Returns:
            ndarray: image in ndarray
        """
        if camera_name not in self.raw_pack_readers:
            pack_paths = self.get_raw_pack_paths(camera_name)
            self.raw_pack_readers[camera_name] = []
            for pack_path in pack_paths:
                self.raw_pack_readers[camera_name].append(ImagePackReader(pack_path))  # noqa E501
        for reader in self.raw_pack_readers[camera_name]:
            image = reader.get_image_by_timestamp(timestamp)
            if image is not None:
                return image
        return None

    def get_camera_by_index(self, camera_name, index, sync=True):
        """get camera image in ndarray by index

        Args:
            camera_name (string): camera name, e.g. "camera_front"
            index (int): index of image
            sync (bool, optional): whether load index from sync or unsync. Defaults to True.  # noqa E501

        Returns:
            ndarray: image in ndarray
        """
        sync_string = "sync" if sync else "unsync"
        timestamp_int = self.attribute[sync_string][camera_name][index]
        return self.get_camera_by_timestamp(camera_name, timestamp_int)

    def get_lidar_by_timestamp(self, timestamp, sensor_name="lidar_top", prefix="Pandar"):
        """get pointcloud in ndarray by timestamp

        Args:
            timestamp (int): timestamp in milliseconds
            sensor_name (string, optional): sensor name. Defaults to "lidar_top". # noqa E501

        Returns:
            ndarray: pointcloud in ndarray (Nx6: x, y, z, intensity, ring, timestamp) # noqa E501
        """
        if sensor_name not in self.raw_pack_readers:
            pack_paths = self.get_raw_pack_paths(sensor_name, prefix=prefix)
            self.raw_pack_readers[sensor_name] = []
            for pack_path in pack_paths:
                self.raw_pack_readers[sensor_name].append(PointcloudPackReader(pack_path))  # noqa E501
        for reader in self.raw_pack_readers[sensor_name]:
            pointcloud = reader.get_pointcloud_by_timestamp(timestamp)
            if pointcloud is not None:
                return pointcloud
        return None

    def get_lidar_by_index(self, index, sensor_name="lidar_top", sync=True, prefix="Pandar"):
        """get pointcloud in ndarray by index

        Args:
            index (int): index of pointcloud
            sensor_name (string, optional): sensor name. Defaults to "lidar_top".
            sync (bool, optional): whether load index from sync or unsync. Defaults to True.

        Returns:
            ndarray: pointcloud in ndarray (Nx6: x, y, z, intensity, ring, timestamp) # noqa E501
        """
        sync_string = "sync" if sync else "unsync"
        timestamp_int = self.attribute[sync_string][sensor_name][index]
        return self.get_lidar_by_timestamp(timestamp_int, sensor_name, prefix=prefix)

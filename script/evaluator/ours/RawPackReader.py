# Copyright 2023 <Copyright hobot>
# @date 15/Feb./2023
# @author Jiaxin Zhang

import numpy as np
import cv2

from tat.matrix.msg.reader import MSGReader, TopicChannel
from tat.matrix.pack_sdk import PackFileReader


class PackReader:
    """Base Wrapper for TAT Pack Reader
    """

    def __init__(self, pack_path, topic, channel=0):
        pack_reader = PackFileReader(pack_path)
        self.topic = topic
        self.channel = channel
        self.reader = MSGReader(
            handle=pack_reader,
            topic_channel=[TopicChannel(self.topic, self.channel)],
            decode_data=True,
        )

    def get_msgs_by_timestamp(self, timestamp):
        timeline = self.reader.topic_timelines._timeline_dict[(self.topic, self.channel)]
        frame_idx_sets = timeline.seek_by_timestamp(
            ts_start=timestamp,
            ts_end=timestamp)
        frame_set = None
        # Sometimes a pack may have multiple timeline segments
        for one_set in frame_idx_sets:
            if len(one_set) != 0:
                frame_set = one_set
        if frame_set is None:
            return None
        # Sometimes a single timestamp query will result to multiple index
        # check msgs timestamp to make sure it returns the correct one
        for selected_frame_idx in frame_set:
            self.reader.SeekByIndex(selected_frame_idx)
            msgs = self.reader.Read()
            msgs_timestamp = msgs[self.topic][self.channel].frame.timestamp
            if msgs_timestamp == timestamp:
                return msgs
        return None


class ImagePackReader(PackReader):
    def __init__(self, pack_path, topic="image", channel=0):
        super().__init__(pack_path, topic, channel)

    def get_image_by_timestamp(self, timestamp):
        """get image in ndarray by timestamp

        Args:
            timestamp (int): timestamp in milliseconds

        Returns:
            ndarray: image in ndarray
        """
        msgs = self.get_msgs_by_timestamp(timestamp)
        if msgs is None:
            return None
        img_msg = msgs[self.topic][self.channel]
        jpg_data = img_msg.get_jpg_data()
        np_data = np.frombuffer(jpg_data[0], dtype=np.uint8)
        image = cv2.imdecode(np_data, cv2.IMREAD_UNCHANGED)
        return image


class PointcloudPackReader(PackReader):
    def __init__(self, pack_path, topic="hesai", channel=0):
        super().__init__(pack_path, topic, channel)

    def get_pointcloud_by_timestamp(self, timestamp):
        """get image in ndarray by timestamp

        Args:
            timestamp (int): timestamp in milliseconds

        Returns:
            ndarray: image in ndarray
        """
        msgs = self.get_msgs_by_timestamp(timestamp)
        if msgs is None:
            return None
        sensor_msg = msgs[self.topic][self.channel]
        array = sensor_msg.data[0]
        array = array.astype(np.double)
        return array

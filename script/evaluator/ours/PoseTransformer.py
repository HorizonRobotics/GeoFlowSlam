# Copyright 2020 <Copyright hobot>
# @date 22/Dec./2020
# @author Jiaxin Zhang

from datetime import datetime
from warnings import warn
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import numpy as np


def invT(transform):
    """inverse a transform matrix without using np.linalg.inv

    Args:
        transform (ndarray): input transform matrix with shape=(4,4)

    Returns:
        ndarray: output transform matrix with shape=(4,4)
    """
    R_Transposed = transform[:3, :3].T
    result = np.eye(4)
    result[:3, :3] = R_Transposed
    result[:3, 3] = -R_Transposed @ transform[:3, 3]
    return result


def SlerpTransform(transform_left, transform_right, ratio):
    """perform linear interpolation between two transform

    Args:
        transform_left (ndarray): shape=(4, 4)
        transform_right (ndarray): shape=(4, 4)
        ratio (float): ration between left and right, must between 0 to

    Returns:
        ndarray: interpolated transform, shape=(4, 4)
    """
    assert 0 <= ratio <= 1, "ratio must between 0 to 1"
    assert transform_left.shape == transform_right.shape == (4, 4), "transform must be ndarray with 4x4"
    transforms = np.concatenate([transform_left[np.newaxis, :3, :3], transform_right[np.newaxis, :3, :3]], axis=0)
    key_rots = R.from_matrix(transforms)
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)
    slerped_rotation = slerp(ratio).as_matrix()
    slerped_transform = transform_left * (1-ratio) + transform_right * ratio
    slerped_transform[:3, :3] = slerped_rotation
    return slerped_transform


class PoseTransformer:
    def __init__(self, euler_order="ZXY", degree=False):
        """a util class for 3D poses(odometry) manipulation,
        including transformation between relative and absoltue,
        different representation of rotation(euler, axis_angle, quaternion)

        Args:
            euler_order (str, optional): euler angle order. Defaults to "ZXY".
            degree (bool, optional): if True use degree rather than radian. Defaults to False.
        """
        self.euler_order = euler_order
        self.degree = degree
        self.relative_rotation = []  # [N-1, 3, 3]
        self.relative_translation = []  # [N-1, 3 , 1]
        self.relative_transform = []  # [N-1 ,4, 4]
        self.absolute_transform = []  # [N ,4, 4]
        self.timestamps = []  # [N ,1]

    def reset(self):
        """clear all poses and timestamps
        """
        self.relative_rotation = []
        self.relative_translation = []
        self.relative_transform = []
        self.absolute_transform = []
        self.timestamps = []

    def from_relative_transform(self, transform_array):
        assert transform_array.shape[1] == 4
        assert transform_array.shape[2] == 4
        self.relative_transform = transform_array
        self.absolute_transform = []

    def from_absolute_transform(self, transform_array):
        assert transform_array.shape[1] == 4
        assert transform_array.shape[2] == 4
        self.absolute_transform = transform_array
        self.__calculate_relative_transform()

    def from_axis_angle(self, axis_angles, absolute):
        """load rotation from axis angle(lie algebra), support absolute(global) and relative(local)

        Args:
            axis_angles (ndarray): shape:(B, 3), batch of axis angle representation
            absolute (bool): if True, axis_angles mean absoltue(global) rotation
        """
        if absolute:
            self.from_absolute_axis_angle(axis_angles)
        else:
            self.from_relative_axis_angle(axis_angles)

    def from_relative_axis_angle(self, axis_angles):
        error_msg = "axis_angles must be np.array in shape [B, 3]"
        assert len(axis_angles.shape) == 2, error_msg
        assert axis_angles.shape[1] == 3, error_msg
        self.relative_rotation = []
        self.absolute_transform = []
        for axis_angle in axis_angles:
            rotation = R.from_rotvec(axis_angle).as_matrix()
            self.relative_rotation.append(rotation)

    def from_absolute_axis_angle(self, axis_angles):
        error_msg = "axis_angles must be np.array in shape [B, 3]"
        assert len(axis_angles.shape) == 2, error_msg
        assert axis_angles.shape[1] == 3, error_msg
        absolute_rotations = R.from_rotvec(axis_angles).as_matrix()
        if len(self.absolute_transform) == 0:
            self.absolute_transform = np.eye(4, dtype=absolute_rotations.dtype)[np.newaxis, :, :]
            self.absolute_transform = np.tile(self.absolute_transform, (absolute_rotations.shape[0], 1, 1))
        else:
            error_msg = "previous stored absolute transform number not matched with input axis angles"
            assert len(self.absolute_transform) == axis_angles.shape[0], error_msg
            self.absolute_transform = np.asarray(self.absolute_transform)
        self.absolute_transform[:, :3, :3] = absolute_rotations
        self.absolute_transform = list(self.absolute_transform)

    def from_absolute_translation(self, translations):
        error_msg = "translations must be np.array in shape [B, 3]"
        assert len(translations.shape) == 2, error_msg
        assert translations.shape[1] == 3, error_msg
        if len(self.absolute_transform) == 0:
            self.absolute_transform = np.eye(4, dtype=translations.dtype)[np.newaxis, :, :]
            self.absolute_transform = np.tile(self.absolute_transform, (translations.shape[0], 1, 1))
        else:
            error_msg = "previous stored absolute transform number not matched with input translations"
            assert len(self.absolute_transform) == translations.shape[0], error_msg
            self.absolute_transform = np.asarray(self.absolute_transform)
        self.absolute_transform[:, :3, 3] = translations
        self.absolute_transform = list(self.absolute_transform)

    def from_relative_quaternion(self, quaternions):
        error_msg = "quaternions must be np.array in shape [B, 4]"
        assert len(quaternions.shape) == 2, error_msg
        assert quaternions.shape[1] == 4, error_msg
        self.relative_rotation = []
        self.absolute_transform = []
        for quaternion in quaternions:
            rotation = R.from_quat(quaternion).as_matrix()
            self.relative_rotation.append(rotation)

    def from_relative_eulers(self, eulers):
        self.relative_rotation = []
        self.absolute_transform = []
        for euler in eulers:
            rotation = R.from_euler(seq=self.euler_order,
                                    angles=euler,
                                    degrees=self.degree).as_matrix()
            self.relative_rotation.append(rotation)

    def from_translation(self, translations, absolute):
        """load translation vectors

        Args:
            translations (ndarray): shape:(B, 3), batch of translation vectors
            absolute (bool): if True, translations mean absolute(global) poses
        """
        if absolute:
            self.from_absolute_translation(translations)
        else:
            self.from_relative_translation(translations)

    def from_relative_translation(self, translations):
        self.relative_translation = []
        self.absolute_transform = []
        for translation in translations:
            self.relative_translation.append(translation)

    def __calculate_relative_transform(self):
        assert len(self.relative_rotation) == len(self.relative_translation)
        for idx in range(len(self.relative_rotation)):
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = self.relative_rotation[idx]
            transform[:3, 3] = self.relative_translation[idx]
            transform = invT(transform)
            self.relative_transform.append(transform)

    def __absolute2relative(self):
        num_frames = len(self.absolute_transform)
        if num_frames == 0:
            raise RuntimeError("please load absolute first,\
                by using loadtxt()")
        self.relative_transform = []
        self.relative_rotation = []
        self.relative_translation = []
        for idx in range(num_frames - 1):
            relative_transform = invT(
                self.absolute_transform[idx + 1]) @ self.absolute_transform[idx]
            self.relative_transform.append(relative_transform)
            self.relative_rotation.append(relative_transform[:3, :3])
            self.relative_translation.append(relative_transform[:3, 3:])

    def __relative2absolute(self):
        if len(self.relative_transform) == 0:
            self.__calculate_relative_transform()
        assert len(self.relative_transform) > 0
        self.absolute_transform = []
        self.absolute_transform.append(np.eye(4, dtype=np.float64))
        for transform in self.relative_transform:
            current_absolute_transform =\
                self.absolute_transform[-1] @ transform
            self.absolute_transform.append(current_absolute_transform)

    def as_quaternions(self, absolute=True):
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        if not absolute:
            raise NotImplementedError("sorry, not yet supported :-(")
        quaternions = []
        for absolute_transform in self.absolute_transform:
            quaternion = R.from_matrix(absolute_transform[:3, :3]).as_quat()
            quaternions.append(quaternion)
        return quaternions

    def as_euler(self, absolute):
        if len(self.relative_transform) == 0 and len(self.absolute_transform) == 0:
            raise RuntimeError("please load data first!")
        if absolute:
            if len(self.absolute_transform) == 0:
                self.__relative2absolute()
            absolute_rotations = np.asarray(self.absolute_transform)[:, :3, :3]
            eulers = R.from_matrix(absolute_rotations).as_euler(
                seq=self.euler_order, degrees=self.degree
            )
        else:
            if len(self.relative_transform) == 0:
                self.__absolute2relative()
            eulers = []
            for relative_transform in self.relative_transform:
                relative_rotation = relative_transform[:3, :3]
                euler = R.from_matrix(relative_rotation).as_euler(
                    seq=self.euler_order, degrees=self.degree)
                eulers.append([euler])
            eulers = np.concatenate(eulers, axis=0)
        return eulers

    def as_axis_angle(self, absolute):
        if len(self.relative_transform) == 0 and len(self.absolute_transform) == 0:
            raise RuntimeError("please load data first!")
        if absolute:
            if len(self.absolute_transform) == 0:
                self.__relative2absolute()
            absolute_rotations = np.asarray(self.absolute_transform)[:, :3, :3]
            axis_angles = R.from_matrix(absolute_rotations).as_rotvec()
        else:
            if len(self.relative_transform) == 0:
                self.__absolute2relative()
            axis_angles = []
            for relative_transform in self.relative_transform:
                relative_rotation = relative_transform[:3, :3]
                axis_angle = R.from_matrix(relative_rotation).as_rotvec()
                axis_angles.append([axis_angle])
            axis_angles = np.concatenate(axis_angles, axis=0)
        return axis_angles

    def as_axisangle(self, absolute):
        warn("Warning(Deprecation): as_axisangle is renamed to as_axis_angle, please consider update")
        return self.as_axis_angle(absolute=absolute)

    def as_translations(self, absolute):
        if len(self.relative_transform) == 0 and len(self.absolute_transform) == 0:
            raise RuntimeError("please load data first!")
        if absolute:
            if len(self.absolute_transform) == 0:
                self.__relative2absolute()
            translations = []
            for absolute_transform in self.absolute_transform:
                translation = absolute_transform[:3, 3]
                translations.append(translation)
            return np.asarray(translations)
        else:
            if len(self.relative_transform) == 0:
                self.__absolute2relative()
            translations = []
            for relative_transform in self.relative_transform:
                relative_translation = relative_transform[:3, 3]
                translations.append([relative_translation])
            translations = np.concatenate(translations, axis=0)
            return translations

    def as_trans_quat(self, absolute=True):
        quaternions = np.asarray(self.as_quaternions(absolute=absolute))
        translations = np.asarray(self.as_translations(absolute=absolute))
        trans_quat = np.concatenate((translations, quaternions), axis=1)
        return trans_quat

    def as_transform(self, absolute=True):
        """return Transform Matrixs

        Args:
            absolute (bool, optional):
            whether return absolute transform w.r.t. world coordinate,
            or return relative transform between last and current frame.
            Defaults to True.

        Returns:
            [np.array]: [Transform matrixs in shape of (B, 4, 4)]
        """
        if absolute:
            if len(self.absolute_transform) == 0:
                self.__relative2absolute()
            return np.asarray(self.absolute_transform)
        else:
            return np.asarray(self.relative_transform)

    def normalize2origin(self):
        """normalize absolute transforms to the start point
            i.e transform[0] = Indentity Matrix
        """

        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        origin_transform_inv = invT(self.absolute_transform[0])
        result_transform = []
        for transform in self.absolute_transform:
            result_transform.append(origin_transform_inv @ transform)
        self.absolute_transform = result_transform

    def __dumparray_tum(self):
        if(len(self.relative_transform) == 0 and len(self.absolute_transform) == 0 and len(self.relative_translation) == 0):
            raise RuntimeError("No poses found, pleas load poses first")
        if(self.timestamps.shape[0] == 0):
            raise RuntimeError("No timestamps found, pleas load timestamps first")
        if(len(self.absolute_transform) == 0):
            self.__relative2absolute()
        if self.timestamps.shape[0] == len(self.absolute_transform):
            # assuming # of timestamps = N+1, correspond to absolute transform
            pass
        elif self.timestamps.shape[0] + 1 == len(self.absolute_transform):
            # assuming # of timestamps = N, drop first element from absolute transform
            self.absolute_transform = self.absolute_transform[1:]
        else:
            error_msg = "num of timestamps = {} while num of absolute transform = {}\n".format(self.timestamps.shape[0], len(self.absolute_transform))
            error_msg += "they should be equal or num of timestamps +1 = num of absolute transform"
            raise RuntimeError(error_msg)
        trans_quat = self.as_trans_quat(absolute=True)
        tum_array = np.concatenate((self.timestamps, trans_quat), axis=1)
        return tum_array

    def dumparray(self, style="tum"):
        if style == "tum":
            return self.__dumparray_tum()
        else:
            raise NotImplementedError(
                "style {} not supported yet.\nCurrently support [tum]".format(style))

    def load_timestamp(self, timestamps, style="unix", relative=True):
        if style == "unix":
            self.__load_timestamp_unix(timestamps)
        elif style == "kitti":
            self.__load_timestamp_kitti(timestamps)
        else:
            raise NotImplementedError(
                "style {} not supported yet.\nCurrently support [unix(tum), kitti]".format(style))

    def __load_timestamp_kitti(self, timestamps):
        unix_timstamps = []
        for timestamp in timestamps:
            dt = datetime.strptime(timestamp[:-4], '%Y-%m-%d %H:%M:%S.%f').timestamp()
            unix_timstamps.append(dt)
        self.__load_timestamp_unix(unix_timstamps)

    def __load_timestamp_unix(self, timestamps):
        if isinstance(timestamps, list):
            self.timestamps = np.expand_dims(np.asarray(timestamps), axis=-1)
        else:
            # assmuing numpy array
            assert(timestamps.shape[0] > 0)
            if (len(timestamps.shape) == 1):
                self.timestamps = np.expand_dims(timestamps, axis=-1)
            elif (len(timestamps.shape) == 2):
                self.timestamps = timestamps
            else:
                raise RuntimeError("input timestamp shape {} incorrect!".format(timestamps.shape))

    def loadarray(self, array, style="tum"):
        """load poses from ndarray in various format

        Args:
            array (ndarray): ndarray containing poses and timestamps information
            style (str, optional): could be any of ["tum", "kitti", "asl"]. Defaults to "tum".
            please refer to https://github.com/MichaelGrupp/evo/wiki/Formats
            for more details.

        Raises:
            NotImplementedError: if style not in ["tum", "kitti", "asl"]
        """
        self.reset()
        if style == "tum":
            self.__loadarray_tum(array)
        elif style == "kitti":
            self.__loadarray_kitti(array)
        elif style == "asl":
            self.__loadarray_asl(array)
        else:
            raise NotImplementedError(
                "style {} not supported yet.\nCurrently support [tum, kitit, asl]".format(style))

    def __loadarray_kitti(self, array):
        input_pose = array
        assert(input_pose.shape[1] == 12)
        length = input_pose.shape[0]
        input_pose = input_pose.reshape(-1, 3, 4)
        bottom = np.zeros((length, 1, 4))
        bottom[:, :, -1] = 1
        transforms = np.concatenate((input_pose, bottom), axis=1)
        self.absolute_transform = transforms
        self.__absolute2relative()

    def __loadarray_tum(self, array):
        assert array.shape[1] == 8
        self.timestamps = array[:, 0:1]
        length = array.shape[0]
        absolute_transforms = np.zeros((length, 4, 4))
        absolute_transforms[:, 3, 3] = 1
        absolute_transforms[:, :3, :3] = R.from_quat(array[:, 4:8]).as_matrix()
        absolute_transforms[:, :3, 3] = array[:, 1:4]
        self.absolute_transform = list(absolute_transforms)
        self.__absolute2relative()

    def __loadarray_asl(self, array):
        """
        load poses from asl format array
        ASL (or better known as EuRoC MAV) pose format

        Args:
            array (numpy array with shape (N, 17)):
            timestamp	 p_RS_R_x [m]	 p_RS_R_y [m]	 p_RS_R_z [m]
            q_RS_w []	 q_RS_x []	 q_RS_y []	 q_RS_z []
            v_RS_R_x [m s^-1]	 v_RS_R_y [m s^-1]	 v_RS_R_z [m s^-1]
            b_w_RS_S_x [rad s^-1]	 b_w_RS_S_y [rad s^-1]	 b_w_RS_S_z [rad s^-1]
            b_a_RS_S_x [m s^-2]	 b_a_RS_S_y [m s^-2]	 b_a_RS_S_z [m s^-2]

        """
        assert array.shape[1] == 17
        length = array.shape[0]
        timestamps = array[:, 0] * 1e-9  # nanoseconds to seconds
        quats = array[:, [5, 6, 7, 4]]  # quat(w x y z) to quat(x y z w)
        transforms = np.zeros((length, 4, 4))
        transforms[:, 3, 3] = 1
        transforms[:, :3, :3] = R.from_quat(quats).as_matrix()  # rotation
        transforms[:, :3, 3] = array[:, 1:4]  # translation
        self.absolute_transform = list(transforms)
        self.__absolute2relative()
        self.timestamps = np.expand_dims(np.array(timestamps), axis=1)

    def get_timestamps(self):
        if len(self.timestamps) == 0:
            raise RuntimeError("please load timestamps first, from loadtxt()")
        return self.timestamps

    def rotate(self, extrinsic):
        """This function is deprecated as it may lead to misunderstanding.
            For example, the pose sequence is loaded and describes sensorA_i to sensorA_0.
            rotate(sensorB_2_sensorA) will give you sensorB_i to sensorA_0.
            transform(sensorA_2_sensorB) will give you sensorB_i to sensorB_0.
            e.g
                self.absolute_transform describe
                sensor A's poses w.r.t world coordinate,

                extrinsic describe transform from B to A,
                then by doing rotate(),

                self.absolute_transform will describe
                sensor B's poses w.r.t world coordinate
            If you want excatly the behavior of this function, please consider right_rotate


        Args:
            extrinsic (np.array: transform matrix in shape (4, 4)
        """
        warn("Warning(Deprecation): rotate function may lead misunderstanding\nPlease consider using transform()")
        assert extrinsic.shape == (4, 4)
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        result_transforms = []
        for transform in self.absolute_transform:
            result_transform = transform @ extrinsic
            result_transforms.append(result_transform)
        self.absolute_transform = result_transforms

    def left_rotate(self, extrinsic):
        """Apply extrinsic to the left of every absolute_transform.
            e.g. new_transform_i = extrinsic @ old_transform_i for every i
            Note that this function does not imply any physical or geometrical meanings.
            Use this as a helper function if you know exactly what you want.
            If you need to transform a pose trajectory to another sensor, consider using transform().

        Args:
            extrinsic (ndarray): shape (4, 4)
        """
        assert extrinsic.shape == (4, 4)
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        result_transforms = []
        for transform in self.absolute_transform:
            result_transform = extrinsic @ transform
            result_transforms.append(result_transform)
        self.absolute_transform = result_transforms

    def right_rotate(self, extrinsic):
        """Apply extrinsic to the right of every absolute_transform.
            e.g. new_transform_i = old_transform_i @ extrinsic for every i
            Note that this function does not imply any physical or geometrical meanings.
            Use this as a helper function if you know exactly what you want.
            If you need to transform a pose trajectory to another sensor, consider using transform().

        Args:
            extrinsic (ndarray): shape (4, 4)
        """
        assert extrinsic.shape == (4, 4)
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        result_transforms = []
        for transform in self.absolute_transform:
            result_transform = transform @ extrinsic
            result_transforms.append(result_transform)
        self.absolute_transform = result_transforms

    def transform(self, extrinsic):
        """Transform a sequence of pose from sensorA to sensorB.
            Suppose the self.absolute_transform describe a pose sequence in
            sensorA coordinate system. extrinsic describe 4x4 transform matrix
            from sensorA to sensorB. By applying this function,
            self.absolute_transform will represent pose sequence in sensorB
            coordinate system.

        Args:
            extrinsic (ndarray): 4x4 transform matrix from sensorA to sensorB.
            To be specifically, if a point X with coordinate Xa = [xa, ya, za].T
            is expressed in sensorA coordinate system, extrinsic @ Xa is the
            point coordinate in sensorB coordinate system.
        """

        assert extrinsic.shape == (4, 4)
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()
        result_transforms = []
        for transform in self.absolute_transform:
            result_transform = extrinsic @ transform @ invT(extrinsic)
            result_transforms.append(result_transform)
        self.absolute_transform = result_transforms

    def __sort_relative_transform_by_timestamps(self):
        if self.timestamps.shape[0] != len(self.relative_transform):
            raise RuntimeError("# of timestamps = {} but # relative transform = {}".format(self.timestamps.shape[0], len(self.relative_transform)))
        ts_ind = np.argsort(self.timestamps[:, 0])
        self.relative_transform = list(np.asarray(self.relative_transform)[ts_ind])
        self.timestamps = self.timestamps[ts_ind]

    def __sort_absolute_transform_by_timestamps(self):
        ts_ind = np.argsort(self.timestamps[:, 0])
        self.absolute_transform = list(np.asarray(self.absolute_transform)[ts_ind])
        self.timestamps = self.timestamps[ts_ind]

    def sort_by_timestamps(self):
        if self.timestamps.shape[0] < 2:
            raise RuntimeError("there are only {} timestamps".format(self.timestamps.shape[0]))

        if len(self.absolute_transform) == self.timestamps.shape[0]:
            self.__sort_absolute_transform_by_timestamps()
        elif self.timestamps.shape[0] == len(self.relative_rotation) and self.timestamps.shape[0] == len(self.relative_translation):
            self.__calculate_relative_transform()
            self.__sort_relative_transform_by_timestamps()
        elif self.timestamps.shape[0] == len(self.relative_transform):
            self.__sort_relative_transform_by_timestamps()
        else:
            raise NotImplementedError("whooops! not supported yet")

    def seek_by_timestamp(self, query_time: float, t_max_diff: float, interpolate=False):
        """Seek transform by given query_time. There are two mode supported:
            interpolate=True:
                query_time is legal within [timestamps[0], timestamps[1]]
                if query_time fall between (timestamps[i], timestamps[i+1]), then assume
                timestamps[i+1] - timestamps[i] < t_max_diff, otherwise raise RuntimeError

            interpolate=False:
                query_time is legal within (timestamps[0] - t_max_diff, timestamps[1] + t_max_diff)
                if query_time fall between (timestamps[i], timestamps[i+1]), then assume
                min(timestamps[i+1] - query_time, query_time - timestamps[i]) < t_max_diff.
                Note that in this mode, timestamps could be very sparse, allowing every query_time be very close
                or equal to one of timestamps.

        Args:
            query_time (float): must be within the loaded timestamps
            t_max_diff (float): maximum difference time allowed for any consecutive timestamps
            interpolate (bool, optional): whether to use interpolation or just simply find the transform with nearest timestamp. Defaults to False.

        Returns:
            ndarray: transform with shape=(4, 4)
        """
        assert isinstance(query_time, float), f"query_time must be float, not {type(query_time)}"
        assert isinstance(t_max_diff, float), f"t_max_diff must be float, not {type(t_max_diff)}"
        if(len(self.relative_transform) == 0 and len(self.absolute_transform) == 0 and len(self.relative_translation) == 0):
            raise RuntimeError("No poses found, pleas load poses first")
        if(self.timestamps.shape[0] == 0):
            raise RuntimeError("No timestamps found, pleas load timestamps first")
        if(len(self.absolute_transform) == 0):
            self.__relative2absolute()

        assert np.all(self.timestamps[1:, 0] >= self.timestamps[:-1, 0]), "timestamps must be sorted"

        # check if query_time match any timestamp in self.timestamps exactly
        equal_timestamp_index = np.where(np.isclose(self.timestamps[:, 0], query_time, rtol=1e-20, atol=1e-9))[0]
        if equal_timestamp_index.size > 0:
            return self.absolute_transform[equal_timestamp_index[0]]

        right_index = np.searchsorted(self.timestamps[:, 0], query_time, side="left")
        left_index = right_index - 1

        if interpolate:
            if right_index >= self.timestamps.shape[0]:
                raise RuntimeError("query_time is out of range.")
            if right_index == 0 and -1e-9 < (query_time - self.timestamps[0]) < 0:
                right_index = 1
                left_index = 0
            elif query_time - self.timestamps[0] < -1e-9:
                raise RuntimeError("query_time is out of range.")
            time_diff = self.timestamps[right_index] - self.timestamps[left_index]
            if time_diff > t_max_diff:
                raise RuntimeError(f"time_diff = {time_diff} is greater than t_max_diff {t_max_diff}")
            ratio = (query_time - self.timestamps[left_index]) / time_diff
            output_transform = SlerpTransform(self.absolute_transform[left_index], self.absolute_transform[right_index], ratio)
        else:
            left_time_diff = query_time - self.timestamps[left_index] if left_index >= 0 else float("inf")
            right_time_diff = self.timestamps[right_index] - query_time if right_index < self.timestamps.shape[0] else float("inf")
            time_diff = min(left_time_diff, right_time_diff)[0]
            if time_diff > t_max_diff:
                raise RuntimeError(f"time_diff = {time_diff} is greater than t_max_diff {t_max_diff}")
            query_index = left_index if left_time_diff < right_time_diff else right_index
            output_transform = self.absolute_transform[query_index]

        return output_transform
    def seek_by_timestamps(self, query_times: list, t_max_diff: float, interpolate=True):
        """
        根据给定的时间戳数组查询对应的姿态数据。支持两种模式：
            interpolate=True:
                query_time 在 [timestamps[0], timestamps[1]] 范围内是合法的
                如果 query_time 落在 (timestamps[i], timestamps[i+1]) 之间，则假设
                timestamps[i+1] - timestamps[i] < t_max_diff，否则抛出 RuntimeError

            interpolate=False:
                query_time 在 (timestamps[0] - t_max_diff, timestamps[1] + t_max_diff) 范围内是合法的
                如果 query_time 落在 (timestamps[i], timestamps[i+1]) 之间，则假设
                min(timestamps[i+1] - query_time, query_time - timestamps[i]) < t_max_diff。
                注意在这种模式下，时间戳可以非常稀疏，允许每个 query_time 非常接近或等于某个时间戳。

        参数:
            query_times (list): 时间戳数组
            t_max_diff (float): 允许的最大时间差
            interpolate (bool, optional): 是否使用插值，默认为 False

        返回:
            list: 对应的姿态数据数组，每个姿态数据的形状为 (4, 4)
        """
        assert isinstance(query_times, list), f"query_times must be a list, not {type(query_times)}"
        assert all(isinstance(query_time, float) for query_time in query_times), "All query_times must be float"
        assert isinstance(t_max_diff, float), f"t_max_diff must be float, not {type(t_max_diff)}"
        if len(self.relative_transform) == 0 and len(self.absolute_transform) == 0 and len(self.relative_translation) == 0:
            raise RuntimeError("No poses found, please load poses first")
        if self.timestamps.shape[0] == 0:
            raise RuntimeError("No timestamps found, please load timestamps first")
        if len(self.absolute_transform) == 0:
            self.__relative2absolute()

        assert np.all(self.timestamps[1:, 0] >= self.timestamps[:-1, 0]), "timestamps must be sorted"

        results = []
        for query_time in query_times:
            # check if query_time match any timestamp in self.timestamps exactly
            equal_timestamp_index = np.where(np.isclose(self.timestamps[:, 0], query_time, rtol=1e-20, atol=1e-9))[0]
            if equal_timestamp_index.size > 0:
                results.append(self.absolute_transform[equal_timestamp_index[0]])
                continue

            right_index = np.searchsorted(self.timestamps[:, 0], query_time, side="left")
            left_index = right_index - 1

            if interpolate:
                if right_index >= self.timestamps.shape[0]:
                    print(query_time)
                    raise RuntimeError("query_time is out of range.")
                if right_index == 0 and -1e-9 < (query_time - self.timestamps[0]) < 0:
                    right_index = 1
                    left_index = 0
                elif query_time - self.timestamps[0] < -1e-9:
                    print(query_time-self.timestamps[0])
                    raise RuntimeError("query_time is out of range.")
                time_diff = self.timestamps[right_index] - self.timestamps[left_index]
                if time_diff > t_max_diff:
                    print(np.float64(self.timestamps[right_index]))
                    print(np.float64(self.timestamps[left_index]))
                    raise RuntimeError(f"time_diff = {time_diff} is greater than t_max_diff {t_max_diff}")
                ratio = (query_time - self.timestamps[left_index]) / time_diff
                output_transform = SlerpTransform(self.absolute_transform[left_index], self.absolute_transform[right_index], ratio)
            else:
                left_time_diff = query_time - self.timestamps[left_index] if left_index >= 0 else float("inf")
                right_time_diff = self.timestamps[right_index] - query_time if right_index < self.timestamps.shape[0] else float("inf")
                time_diff = min(left_time_diff, right_time_diff)
                if time_diff > t_max_diff:
                    raise RuntimeError(f"time_diff = {time_diff} is greater than t_max_diff {t_max_diff}")
                query_index = left_index if left_time_diff < right_time_diff else right_index
                output_transform = self.absolute_transform[query_index]

            # output_transform转四元数
            quaternions = []
            translation = output_transform[:3, 3]
            # print(translation)
            rotation = R.from_matrix(output_transform[:3, :3]).as_quat()
            # print(rotation)
            # print(query_time)
            # quaternions.append([query_time,translation, rotation])
            # quaternions.append(np.concatenate((translation, rotation)))
            # print(quaternions)
            # query_time translation rotation写成一行

            results.append(np.concatenate(([query_time], translation, rotation)))

        return results
    def transform2quaternion(self, poses):
        """
        Args:
            poses (dict): {idx: 4x4 array}
        Returns:
            quaternions (numpy array): numpy arrays（Nx7）
            for quaternions and translation without index[T,Q]
        """
        quaternions = []
        for idx in sorted(poses.keys()):
            pose = poses[idx]
            translation = pose[:3, 3]
            rotation = R.from_matrix(pose[:3, :3]).as_quat()
            quaternions.append(np.concatenate((translation, rotation)))
        return np.asarray(quaternions)
#!/usr/bin/env python
# Copyright 2020 <Copyright hobot>
# @date 20/Nov/2020
# @author Jiaxin Zhang
# modified from https://github.com/Huangying-Zhan/kitti-odom-eval

from scipy.spatial.transform import Rotation as R
import numpy as np
import copy
import json


class PoseEvaluator():
    def __init__(self, alignment, length=[100, 200, 300, 400, 500, 600, 700, 800], min_matches=10, max_t_diff=0.05, scale=1.0, offset=0):
        """Evaluate Odom in Kitti Benchmark style, RTE, RRE, e.g.
           Derived from github.com/eric-yyjau/kitti-odom-eval
        Args:
            alignment (string): could be 7dof, 6dof, scale or None
            length (list): length of sequence for averaging assessment, default is KITTI standard
            min_matches (int, optional): [min mathhes for timestamps]
            max_t_diff (float, optional): [max time in second for match]
            offset (int, optional): [offset in second for two traj]
        """
        self.lengths = length
        self.num_lengths = len(self.lengths)
        self.min_matches = min_matches
        self.alignment = alignment
        self.max_t_diff = max_t_diff
        self.offset = offset
        self.scale = scale
        if self.alignment != "6dof" and self.scale != 1.0:
            raise RuntimeError("scale = {} can only be used with 6dof alignment".format(scale))
        self.units = {}
        self.units["scale"] = ""
        self.units["quaternion"] = "(x, y, z, w)"
        self.units["translation"] = "(x, y, z) meters"
        self.units["RTE"] = "m/10m"
        self.units["RRE"] = "deg/10m"
        self.units["EulerRoll"] = "deg/10m"
        self.units["EulerPitch"] = "deg/10m"
        self.units["EulerYaw"] = "deg/10m"
        self.units["ATE"] = "meters"
        self.units["RRE_m"] = "deg/m"
        self.units["RRE_deg"] = "deg"
        self.units["ITE"] = "meters/s"
        self.units["IRE"] = "deg/s"
        self.units["instant_roll"] = "deg/s"
        self.units["instant_pitch"] = "deg/s"
        self.units["instant_yaw"] = "deg/s"

    def quaternion2transform(self, quaternions):
        """
        Args:
            quaternions (numpy array): numpy arrays（Nx7）
            for quaternions and translation without index[T,Q]
        Returns:
            poses (dict): {idx: 4x4 array}
        """
        last_row = np.zeros((1, 4))
        last_row[0, 3] = 1
        poses = {}
        for idx, pose in enumerate(quaternions):
            translation = pose[:3][:, np.newaxis]
            rotation = R.from_quat(pose[3:]).as_matrix()
            transform = np.concatenate((rotation, translation), axis=1)
            transform = np.concatenate((transform, last_row), axis=0)
            poses[idx] = transform
        return poses

    def scale_lse_solver(self, X, Y):
        """Least-sqaure-error solver
        Compute optimal scaling factor so that s(X)-Y is minimum
        Args:
            X (KxN array): current data
            Y (KxN array): reference data
        Returns:
            scale (float): scaling factor
        """
        scale = np.sum(X * Y) / np.sum(X**2)
        return scale

    def associate(self, first_list, second_list):
        """
        Associate two dictionaries of (stamp,data).
        As the time stamps never match exactly, we aim
        to find the closest match for every input tuple.
        Input:
        first_list -- first dictionary of (stamp,data) tuples
        second_list -- second dictionary of (stamp,data) tuples
        Output:
        matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

        """
        first_keys = list(first_list.keys())
        second_keys = list(second_list.keys())
        first_keys.sort()
        second_keys.sort()
        potential_matches = [(abs(a - (b + self.offset)), a, b)
                             for a in first_keys for b in second_keys
                             if abs(a - (b + self.offset)) < self.max_t_diff]
        potential_matches.sort()
        matches = []
        for diff, a, b in potential_matches:
            if a in first_keys and b in second_keys:
                first_keys.remove(a)
                second_keys.remove(b)
                matches.append((a, b))

        matches.sort()
        return matches

    def umeyama_alignment(self, x, y, with_scale=False):
        """
        Computes the least squares solution parameters of an Sim(m) matrix
        that minimizes the distance between a set of registered points.
        Umeyama, Shinji: Least-squares estimation of transformation parameters
                        between two point patterns. IEEE PAMI, 1991
        :param x: mxn matrix of points, m = dimension, n = nr. of data points
        :param y: mxn matrix of points, m = dimension, n = nr. of data points
        :param with_scale: set to True to align also the scale
            (default: 1.0 scale)
        :return: r, t, c - rotation matrix, translation vector and scale factor
        """
        if x.shape != y.shape:
            assert False, "x.shape not equal to y.shape"

        # m = dimension, n = nr. of data points
        m, n = x.shape

        # means, eq. 34 and 35
        mean_x = x.mean(axis=1)
        mean_y = y.mean(axis=1)

        # variance, eq. 36
        # "transpose" for column subtraction
        sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

        # covariance matrix, eq. 38
        outer_sum = np.zeros((m, m))
        for i in range(n):
            outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
        cov_xy = np.multiply(1.0 / n, outer_sum)

        # SVD (text betw. eq. 38 and 39)
        u, d, v = np.linalg.svd(cov_xy)

        # S matrix, eq. 43
        s = np.eye(m)
        if np.linalg.det(u) * np.linalg.det(v) < 0.0:
            # Ensure a RHS coordinate system (Kabsch algorithm).
            s[m - 1, m - 1] = -1

        # rotation, eq. 40
        r = u.dot(s).dot(v)

        # scale & translation, eq. 42 and 41
        c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
        t = mean_y - np.multiply(c, r.dot(mean_x))

        return r, t, c

    def array2dict(self, array):
        result_dict = {}
        for line in array:
            result_dict[line[0]] = line[1:]
        return result_dict

    def load_poses(self, pred_array, gt_array):
        pred_array[:, 1:3] *= self.scale
        pred_dict = self.array2dict(pred_array)
        gt_dict = self.array2dict(gt_array)

        matches = self.associate(gt_dict, pred_dict)
        if len(matches) < self.min_matches:
            print("found {} matches".format(len(matches)))
            error_msg = """
                Couldn't find matching timestamp pairs between
                groundtruth and estimated trajectory!
                Did you choose the correct sequence?
                Or try to set a larger t_max_diff.
                """
            raise RuntimeError(error_msg)
        gt = np.asarray([[float(value) for value in gt_dict[a][:]]
                         for a, b in matches])
        pred = np.asarray([[float(value) for value in pred_dict[b][:]]
                           for a, b in matches])
        pred_dict = self.quaternion2transform(pred)
        gt_dict = self.quaternion2transform(gt)
        time_diff = matches[-1][0] - matches[0][0]
        return pred_dict, gt_dict, time_diff

    def trajectory_distances(self, poses):
        """Compute distance for each pose w.r.t frame-0
        Args:
            poses (dict): {idx: 4x4 array}
        Returns:
            dist (float list): distance of each pose w.r.t frame-0
        """
        dist = [0]
        sort_frame_idx = sorted(poses.keys())
        for i in range(len(sort_frame_idx) - 1):
            cur_frame_idx = sort_frame_idx[i]
            next_frame_idx = sort_frame_idx[i + 1]
            P1 = poses[cur_frame_idx]
            P2 = poses[next_frame_idx]
            dx = P1[0, 3] - P2[0, 3]
            dy = P1[1, 3] - P2[1, 3]
            dz = P1[2, 3] - P2[2, 3]
            dist.append(dist[i] + np.sqrt(dx**2 + dy**2 + dz**2))
        return dist

    def rpy_error(self, pose_error):
        rpy = np.abs(
            R.from_matrix(pose_error[:3, :3]).as_euler('zxy', degrees=False))
        return rpy[0], rpy[1], rpy[2]

    def rotation_error(self, pose_error):
        """Compute rotation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            rot_error (float): rotation error
        """
        a = pose_error[0, 0]
        b = pose_error[1, 1]
        c = pose_error[2, 2]
        d = 0.5 * (a + b + c - 1.0)
        rot_error = np.arccos(max(min(d, 1.0), -1.0))
        return rot_error

    def translation_error(self, pose_error):
        """Compute translation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            trans_error (float): translation error
        """
        dx = pose_error[0, 3]
        dy = pose_error[1, 3]
        dz = pose_error[2, 3]
        trans_error = np.sqrt(dx**2 + dy**2 + dz**2)
        return trans_error

    def last_frame_from_segment_length(self, dist, first_frame, length):
        """Find frame (index) that away from the first_frame with
        the required distance
        Args:
            dist (float list): distance of each pose w.r.t frame-0
            first_frame (int): start-frame index
            length (float): required distance
        Returns:
            i (int) / -1: end-frame index. if not found return -1
        """
        for i in range(first_frame, len(dist), 1):
            if dist[i] > (dist[first_frame] + length):
                return i
        return -1

    def calc_sequence_errors(self, poses_gt, poses_result):
        """calculate sequence error
        Args:
            poses_gt (dict): {idx: 4x4 array}, ground truth poses
            poses_result (dict): {idx: 4x4 array}, predicted poses
        Returns:
            err (list list):
                [first_frame, rotation error, translation error, length, speed]
                - first_frame: frist frame index
                - rotation error: rotation error per length
                - translation error: translation error per length
                - length: evaluation trajectory length
                - speed: car speed (#FIXME: 10FPS is assumed)
        """
        err = []
        dist = self.trajectory_distances(poses_gt)
        # print("dist: ", dist)
        self.step_size = 1

        for first_frame in range(0, len(poses_gt), self.step_size):
            for i in range(self.num_lengths):
                len_ = self.lengths[i]
                # print("len: ", len_)
                last_frame = self.last_frame_from_segment_length(
                    dist, first_frame, len_)
                # print("first_frame: ", first_frame, "last_frame: ", last_frame)

                # Continue if sequence not long enough
                if last_frame == -1 or \
                        not(last_frame in poses_result.keys()) or \
                        not(first_frame in poses_result.keys()):
                    # print("skip")
                    # print(first_frame, last_frame)
                    continue

                # compute rotational and translational errors
                pose_delta_gt = np.dot(np.linalg.inv(poses_gt[first_frame]),
                                       poses_gt[last_frame])
                pose_delta_result = np.dot(
                    np.linalg.inv(poses_result[first_frame]),
                    poses_result[last_frame])
                pose_error = np.dot(np.linalg.inv(pose_delta_result),
                                    pose_delta_gt)

                r_err = self.rotation_error(pose_error)
                # print("r_err: ", r_err)
                roll_err, pitch_err, yaw_err = self.rpy_error(pose_error)
                t_err = self.translation_error(pose_error)
                # print(last_frame-first_frame, t_err)

                # compute speed
                num_frames = last_frame - first_frame + 1.0
                speed = len_ / (0.1 * num_frames)

                err.append([
                    first_frame, r_err / len_, t_err / len_, len_, speed,
                    roll_err / len_, pitch_err / len_, yaw_err / len_
                ])
        return err

    def save_sequence_errors(self, err, file_name):
        """Save sequence error
        Args:
            err (list list): error information
            file_name (str): txt file for writing errors
        """
        fp = open(file_name, 'w')
        for i in err:
            line_to_write = " ".join([str(j) for j in i])
            fp.writelines(line_to_write + "\n")
        fp.close()

    def compute_overall_err(self, seq_err):
        """Compute average translation & rotation errors
        Args:
            png_path (string): path to save plot png
        """
        t_err = 0
        r_err = 0
        roll_err = 0
        pitch_err = 0
        yaw_err = 0

        seq_len = len(seq_err)

        if seq_len > 0:
            for item in seq_err:
                r_err += item[1]
                t_err += item[2]
                roll_err += item[-3]
                pitch_err += item[-2]
                yaw_err += item[-1]
            ave_t_err = t_err / seq_len
            ave_r_err = r_err / seq_len
            ave_roll_err = roll_err / seq_len
            ave_pitch_err = pitch_err / seq_len
            ave_yaw_err = yaw_err / seq_len
            return ave_t_err, ave_r_err, ave_roll_err,\
                ave_pitch_err, ave_yaw_err
        else:
            return 0, 0, 0, 0, 0

    def plot_trajectory(self, plot_mode="xz"):
        """plot 2D trajectory

        Args:
            plot_mode (str, optional): plot axis, must be on of ["xy", "yx",
            "xz", "zx", "yz", "zy"]. Defaults to "xz".

        Returns:
            ndarray: image of trajectory, can be used with cv2.imwrite()
        """
        if len(plot_mode) != 2:
            raise KeyError("plot_mode must be one of [xy, yx, xz, zx, yz, zy]")
        xyz_dict = {"x": 0, "y": 1, "z": 2}
        try:
            a = xyz_dict[plot_mode[0]]
            b = xyz_dict[plot_mode[1]]
        except KeyError:
            raise KeyError("plot_mode must be one of [xy, yx, xz, zx, yz, zy]")

        import matplotlib  # noqa
        matplotlib.use('Agg')  # noqa
        from matplotlib import pyplot as plt  # noqa
        plot_keys = ["Ground Truth", "Ours"]
        fontsize_ = 20

        poses_dict = {}
        poses_dict["Ground Truth"] = self.poses_gt
        poses_dict["Ours"] = self.poses_pred

        fig = plt.figure()
        ax = plt.gca()
        ax.set_aspect('equal')

        for key in plot_keys:
            pose_ab = []
            frame_idx_list = sorted(poses_dict["Ours"].keys())
            for frame_idx in frame_idx_list:
                pose = poses_dict[key][frame_idx]
                pose_ab.append([pose[a, 3], pose[b, 3]])
            pose_ab = np.asarray(pose_ab)
            plt.plot(pose_ab[:, 0], pose_ab[:, 1], label=key)

        plt.legend(loc="upper right", prop={'size': fontsize_})
        plt.xticks(fontsize=fontsize_)
        plt.yticks(fontsize=fontsize_)
        plt.xlabel(f"{plot_mode[0]} (m)", fontsize=fontsize_)
        plt.ylabel(f"{plot_mode[1]} (m)", fontsize=fontsize_)
        fig.set_size_inches(10, 10)

        # matplot to numpy array image
        fig.canvas.draw()
        data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3, ))
        return data

    def plot_error(self):
        """Plot per-length error
        Args:
            png_path (string): path to save plot png
        """
        import matplotlib  # noqa
        matplotlib.use('Agg')  # noqa
        from matplotlib import pyplot as plt  # noqa
        # Translation error
        plot_y = []
        plot_x = []
        for len_ in self.lengths:
            plot_x.append(len_)
            if len(self.avg_segment_errs[len_]) > 0:
                plot_y.append(self.avg_segment_errs[len_][0] * 100)
            else:
                plot_y.append(0)
        fontsize_ = 10
        fig = plt.figure()
        plt.plot(plot_x, plot_y, "bs-", label="Translation Error")
        plt.ylabel('Translation Error (%)', fontsize=fontsize_)
        plt.xlabel('Path Length (m)', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        fig.set_size_inches(5, 5)
        # matplot to numpy array image
        fig.canvas.draw()
        trans_img = np.frombuffer(fig.canvas.tostring_rgb(),
                                  dtype=np.uint8)
        trans_img = trans_img.reshape(fig.canvas.get_width_height()[::-1] +
                                      (3, ))
        plt.close(fig)

        # Rotation error
        plot_y = []
        plot_x = []
        for len_ in self.lengths:
            plot_x.append(len_)
            if len(self.avg_segment_errs[len_]) > 0:
                plot_y.append(self.avg_segment_errs[len_][1] / np.pi * 180 *
                              100)
            else:
                plot_y.append(0)
        fontsize_ = 10
        fig = plt.figure()
        plt.plot(plot_x, plot_y, "bs-", label="Rotation Error")
        plt.ylabel('Rotation Error (deg/100m)', fontsize=fontsize_)
        plt.xlabel('Path Length (m)', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        fig.set_size_inches(5, 5)
        # matplot to numpy array image
        fig.canvas.draw()
        rot_img = np.frombuffer(fig.canvas.tostring_rgb(),
                                dtype=np.uint8)
        rot_img = rot_img.reshape(fig.canvas.get_width_height()[::-1] + (3, ))
        plt.close(fig)
        return trans_img, rot_img

    def compute_segment_error(self, seq_errs):
        """This function calculates average errors for different segment.
        Args:
            seq_errs (list list): list of errs;
                [first_frame, rotation error, translation error, length, speed]
                - first_frame: frist frame index
                - rotation error: rotation error per length
                - translation error: translation error per length
                - length: evaluation trajectory length
                - speed: car speed (#FIXME: 10FPS is assumed)
        Returns:
            avg_segment_errs (dict): {100:[avg_t_err, avg_r_err],...}
        """

        segment_errs = {}
        avg_segment_errs = {}
        for len_ in self.lengths:
            segment_errs[len_] = []

        # Get errors
        for err in seq_errs:
            len_ = err[3]
            t_err = err[2]
            r_err = err[1]
            roll_err = err[-3]
            pitch_err = err[-2]
            yaw_err = err[-1]
            segment_errs[len_].append(
                [t_err, r_err, roll_err, pitch_err, yaw_err])

        # Compute average
        for len_ in self.lengths:
            if segment_errs[len_] != []:
                avg_t_err = np.mean(np.asarray(segment_errs[len_])[:, 0])
                avg_r_err = np.mean(np.asarray(segment_errs[len_])[:, 1])
                avg_roll_err = np.mean(np.asarray(segment_errs[len_])[:, 2])
                avg_pitch_err = np.mean(np.asarray(segment_errs[len_])[:, 3])
                avg_yaw_err = np.mean(np.asarray(segment_errs[len_])[:, 4])
                avg_segment_errs[len_] = [
                    avg_t_err, avg_r_err, avg_roll_err, avg_pitch_err,
                    avg_yaw_err
                ]
            else:
                avg_segment_errs[len_] = []
        return avg_segment_errs

    def compute_ATE(self, gt, pred):
        """Compute RMSE of ATE
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        """
        errors = []

        for i in pred:
            cur_gt = gt[i]
            gt_xyz = cur_gt[:3, 3]
            cur_pred = pred[i]
            pred_xyz = cur_pred[:3, 3]
            align_err = gt_xyz - pred_xyz
            errors.append(np.sqrt(np.sum(align_err**2)))
        ate = np.sqrt(np.mean(np.asarray(errors)**2))
        return ate

    def compute_RPE(self, gt, pred):
        """Compute RPE
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        Returns:
            rpe_trans
            rpe_rot
        """
        trans_errors = []
        rot_errors = []
        for i in list(pred.keys())[:-1]:
            gt1 = gt[i]
            gt2 = gt[i + 1]
            gt_rel = np.linalg.inv(gt1) @ gt2

            pred1 = pred[i]
            pred2 = pred[i + 1]
            pred_rel = np.linalg.inv(pred1) @ pred2
            rel_err = np.linalg.inv(gt_rel) @ pred_rel

            trans_errors.append(self.translation_error(rel_err))
            rot_errors.append(self.rotation_error(rel_err))
        # rpe_trans = np.sqrt(np.mean(np.asarray(trans_errors) ** 2))
        # rpe_rot = np.sqrt(np.mean(np.asarray(rot_errors) ** 2))
        rpe_trans = np.mean(np.asarray(trans_errors))
        rpe_rot = np.mean(np.asarray(rot_errors))
        return rpe_trans, rpe_rot

    def scale_optimization(self, gt, pred):
        """ Optimize scaling factor
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        Returns:
            new_pred (4x4 array dict): predicted poses after optimization
        """
        pred_updated = copy.deepcopy(pred)
        xyz_pred = []
        xyz_ref = []
        for i in pred:
            pose_pred = pred[i]
            pose_ref = gt[i]
            xyz_pred.append(pose_pred[:3, 3])
            xyz_ref.append(pose_ref[:3, 3])
        xyz_pred = np.asarray(xyz_pred)
        xyz_ref = np.asarray(xyz_ref)
        scale = self.scale_lse_solver(xyz_pred, xyz_ref)
        for i in pred_updated:
            pred_updated[i][:3, 3] *= scale
        return pred_updated

    def calculate_instant_error(self, gt, pred):
        num = len(gt)
        assert num == len(pred)
        translation_errs = []
        rotation_errs = []
        roll_errs = []
        pitch_errs = []
        yaw_errs = []
        for i in range(num-1):
            rel_T_gt = np.linalg.inv(gt[i]) @ gt[i+1]
            rel_T_pred = np.linalg.inv(pred[i]) @ pred[i+1]
            rel_T = np.linalg.inv(rel_T_gt) @ rel_T_pred
            translation_errs.append(self.translation_error(rel_T))
            rotation_errs.append(self.rotation_error(rel_T))
            r, p, y = self.rpy_error(rel_T)
            roll_errs.append(r)
            pitch_errs.append(p)
            yaw_errs.append(y)
        translation_errs = np.asarray(translation_errs)
        rotation_errs = np.asarray(rotation_errs)
        roll_errs = np.asarray(roll_errs)
        pitch_errs = np.asarray(pitch_errs)
        yaw_errs = np.asarray(yaw_errs)
        translation_errs = np.mean(np.abs(translation_errs))
        rotation_errs = np.mean(np.abs(rotation_errs))
        roll_errs = np.mean(np.abs(roll_errs))
        pitch_errs = np.mean(np.abs(pitch_errs))
        yaw_errs = np.mean(np.abs(yaw_errs))
        result = {}
        result["ITE"] = translation_errs
        result["IRE"] = rotation_errs
        result["instant_roll"] = roll_errs
        result["instant_pitch"] = pitch_errs
        result["instant_yaw"] = yaw_errs
        return result

    def eval(self, gt_array, pred_array):
        alignment = self.alignment
        result_dict = {}
        poses_pred, poses_gt, time_diff = self.load_poses(pred_array, gt_array)
        frame_rate = float(len(poses_gt))/time_diff
        # Pose alignment to first frame
        idx_0 = sorted(list(poses_pred.keys()))[0]
        pred_0 = poses_pred[idx_0]
        gt_0 = poses_gt[idx_0]
        for cnt in poses_pred:
            poses_pred[cnt] = np.linalg.inv(pred_0) @ poses_pred[cnt]
            poses_gt[cnt] = np.linalg.inv(gt_0) @ poses_gt[cnt]

        if alignment == "scale":
            poses_pred = self.scale_optimization(poses_gt, poses_pred)
        elif alignment == "scale_7dof" or \
                alignment == "7dof" or \
                alignment == "6dof":
            # get XYZ
            xyz_gt = []
            xyz_result = []
            for cnt in poses_pred:
                xyz_gt.append([
                    poses_gt[cnt][0, 3], poses_gt[cnt][1, 3], poses_gt[cnt][2,
                                                                            3]
                ])
                xyz_result.append([
                    poses_pred[cnt][0, 3], poses_pred[cnt][1, 3],
                    poses_pred[cnt][2, 3]
                ])
            xyz_gt = np.asarray(xyz_gt).transpose(1, 0)
            xyz_result = np.asarray(xyz_result).transpose(1, 0)

            r, t, scale = self.umeyama_alignment(xyz_result, xyz_gt,
                                                 alignment != "6dof")
            if self.scale == 1.0:
                result_dict["scale"] = scale
            else:
                result_dict["scale"] = self.scale
            # result_dict["quaternion"] = R.from_matrix(r).as_quat()
            # result_dict["translation"] = t

            align_transformation = np.eye(4)
            align_transformation[:3:, :3] = r
            align_transformation[:3, 3] = t

            for cnt in poses_pred:
                poses_pred[cnt][:3, 3] *= scale
                if alignment == "7dof" or alignment == "6dof":
                    poses_pred[
                        cnt] = align_transformation @ poses_pred[cnt]

        # compute sequence errors
        seq_err = self.calc_sequence_errors(poses_gt, poses_pred)
        # print ("seq_err: ", seq_err)
        # print("seq_err: ", seq_err)
        # Compute segment errors
        avg_segment_errs = self.compute_segment_error(seq_err)

        # compute overall error
        ave_t_err, ave_r_err, avg_roll_err, avg_pitch_err, avg_yaw_err = \
            self.compute_overall_err(seq_err)
        # Compute ATE
        ate = self.compute_ATE(poses_gt, poses_pred)
        # Compute RPE
        rpe_trans, rpe_rot = self.compute_RPE(poses_gt, poses_pred)
        instant_result = self.calculate_instant_error(poses_gt, poses_pred)
        # Compute frame nums
        # print(poses_gt.keys())
        # print(poses_pred.keys())
        common_keys = poses_gt.keys() & poses_pred.keys()
        pose_num = len(common_keys)
        # print("frame num: ", pose_num)

        # result_dict["frame_num"] = pose_num
        # result_dict["all_frame_num"] = len(poses_gt.keys())
        result_dict["RTE"] = ave_t_err*10 
        result_dict["RRE"] = ave_r_err / np.pi * 180*10 
        result_dict["EulerRoll"] = avg_roll_err / np.pi * 180*10
        result_dict["EulerPitch"] = avg_pitch_err / np.pi * 180 *10
        result_dict["EulerYaw"] = avg_yaw_err / np.pi * 180*10
        result_dict["ATE"] = ate
        result_dict["RRE_m"] = rpe_trans
        result_dict["RRE_deg"] = rpe_rot * 180 / np.pi
        result_dict["ITE"] = instant_result["ITE"] * frame_rate
        result_dict["IRE"] = instant_result["IRE"] * frame_rate / np.pi * 180
        # result_dict["instant_roll"] = instant_result["instant_roll"] * frame_rate / np.pi * 180
        # result_dict["instant_pitch"] = instant_result["instant_pitch"] * frame_rate / np.pi * 180
        # result_dict["instant_yaw"] = instant_result["instant_yaw"] * frame_rate / np.pi * 180
        # print(result_dict)
        # save for posible plotting
        self.poses_gt = poses_gt
        self.poses_pred = poses_pred
        self.avg_segment_errs = avg_segment_errs

        return result_dict


def main():
    import argparse
    parser = argparse.ArgumentParser(description="""
        Command line interface for pose evaluation."""
    )
    parser.add_argument(
        '--pred',
        required=True,
        help="pred txt path")
    parser.add_argument(
        '--gt',
        required=True,
        help="gt txt path")
    parser.add_argument(
        '--association',
        required=True,
        help="association txt path")
    parser.add_argument(
        "--result",
        default="./result.json",
        help="result json path")
    parser.add_argument(
        '--alignment',
        default="7dof",
        choices=["7dof", "6dof", "scale", "None"],
        help="alignment methods")
    parser.add_argument(
        '--t_max_diff',
        default=0.1,
        type=float,
        help="maximum diff time in seconds allowed for sync")
    parser.add_argument(
        '--scale',
        default=1.0,
        type=float,
        help="translation scale for 6dof alignment")
    parser.add_argument(
        '--extrinsic',
        default=None,
        type=str,
        help="""extrinsic from the pred-sensor to gt-sensor, e.g camera_front2lidar_top.
                It will look for attribute.json in current folder or parent folder.""")
    args = parser.parse_args()
    print(args)
    pred_array = np.loadtxt(args.pred)
    gt_array = np.loadtxt(args.gt)
    associated_array = np.genfromtxt(args.association)

    # 取出pred_array和associated_array第一列的时间戳
    pred_timestamp = pred_array[:, 0]
    associated_timestamp = associated_array[:, 0]
    # associated_timestamp保留四位小数
    associated_timestamp = np.around(associated_timestamp, decimals=4)
    # pred_timestamp保留四位小数
    # pred_timestamp = np.around(pred_timestamp, decimals=4)
    # pred_timestamp中重复时间戳的个数
    # print(len(pred_timestamp))
    unique_pred_timestamp = np.unique(pred_timestamp)
    # print(len(pred_timestamp))

    # 计算交集
    # common_timestamp = np.intersect1d(associated_timestamp, pred_timestamp)

    if args.extrinsic:
        import DatasetReader, PoseTransformer
        from os.path import exists
        if exists("attribute.json"):
            clip_path = "."
        elif exists("../attribute.json"):
            clip_path = "../"
        dr = DatasetReader(clip_path)
        from_sensor = args.extrinsic.split("2")[0]
        to_sensor = args.extrinsic.split("2")[1]
        pred2gt = dr.get_extrinsic(from_sensor, to_sensor)
        pt = PoseTransformer()
        pt.loadarray(pred_array)
        pt.transform(pred2gt)
        pred_array = pt.dumparray()
    args_length=[10]
    pe = PoseEvaluator(
        alignment=args.alignment,
        length=args_length,
        max_t_diff=args.t_max_diff,
        scale=args.scale
    )
    print("Evaluation results:")
    result_dict = pe.eval(gt_array, pred_array)
    np.set_printoptions(precision=2)
    for key, value in result_dict.items():
        try:
            if key in pe.units:
                print("{}= {:0.2f} {}".format(key.ljust(14), value, pe.units[key]))
            else:
                print("{}= {}".format(key.ljust(14), value))
        except TypeError:
            print(key.ljust(12), " = ", value, " ", pe.units[key])

    result_dict["valid_frame_num"] = len(unique_pred_timestamp)
    result_dict["all_frame_num"] = len(associated_timestamp)
    # Convert any numpy arrays in result_dict to lists for JSON serialization
    for key, value in result_dict.items():
        if isinstance(value, np.ndarray):
            result_dict[key] = value.tolist()

    with open(args.result, 'w') as f:
        json.dump(result_dict, f, indent=4)
if __name__ == "__main__":
    main()
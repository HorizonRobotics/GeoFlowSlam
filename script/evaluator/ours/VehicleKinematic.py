# Copyright 2023 <Copyright hobot>
# @date 23/Apr./2023
# @author Qingjie Wang, Jiaxin Zhang

import numpy as np
from horizon_driving_dataset.PoseTransformer import PoseTransformer, invT
from scipy.spatial.transform import Rotation as R
from os.path import join
import matplotlib  # noqa
matplotlib.use('Agg')  # noqa
from matplotlib import pyplot as plt  # noqa


def vis_traj(data, vis_path, timestamps, abnormal_timestamps):
    fig, axs = plt.subplots(len(data), 1, figsize=(10, 10))
    for i, (name, traj) in enumerate(data.items()):
        axs[i].plot(timestamps, traj, label=name)
        axs[i].scatter(abnormal_timestamps, [traj[timestamps.index(timestamp)] for timestamp in abnormal_timestamps], c='r', marker='x')
        axs[i].legend()
        axs[i].set_xlabel('timestamps')
        axs[i].set_ylabel(name)
    fig.savefig(vis_path)
    plt.close(fig)


class VehicleKinematic:
    def __init__(self,
                 pose_tum_array=None,
                 wheelbase: float = 2.5,  # meters
                 static_threshold: float = 0.5,  # m/s
                 steering_threshold: float = 28.648,  # degree
                 slipping_threshold: float = 28.648,  # degree
                 vertical_vel_threshold: float = 0.5,  # m/s
                 pitch_rate_threshold: float = 5.73  # deg/s
                 ):
        """Detect abnormal poses based on KBM(Kinematic Biciycle Model).

        Args:
            pose_tum_array (ndarray, optional): input pose in tum array (must be vcs coordinate!). Defaults to None.
            wheelbase (float, optional): Distance in meters between the centers of the front and rear wheels. Defaults to 2.5.
            static_threshold (float, optional): Threshold for velocity to detect static. Defaults to 0.5 m/s.
            steering_threshold (float, optional): Threshold for steering angle to detect steering. Defaults to 28.648 deg.
            slipping_threshold (float, optional): Threshold for slipping angle allowed in the pose. Defaults to 28.648 deg.
            vertical_vel_threshold (float, optional): Threshold for vertical velocity allowed in the pose. Defaults to 0.5 m/s.
            pitch_rate_threshold (float, optional): Threshold for pitch rate allowed in the pose. Defaults to 5.73 deg/s.
        """
        if pose_tum_array is not None:
            self.load_pose(pose_tum_array)

        self.wheelbase = wheelbase
        self.static_threshold = static_threshold
        self.steering_threshold = steering_threshold
        self.slipping_threshold = slipping_threshold
        self.vertical_vel_threshold = vertical_vel_threshold
        self.pitch_rate_threshold = pitch_rate_threshold

    def load_pose(self, pose_tum_array):
        """load pose from tum array.

        Args:
            pose_tum_array (ndarray): input pose in tum array (must be vcs coordinate!).
        """
        self.pose_tum_array = pose_tum_array
        pt = PoseTransformer(euler_order="zyx")  # vcs
        pt.loadarray(self.pose_tum_array)
        pt.normalize2origin()
        self.relative_transforms = pt.as_transform(absolute=False)

    def plot_trajectory(self, vis_dir):
        """plot trajectory including pose and state.

        Args:
            vis_dir (string): output directory. Multiple png images will be saved in this directory.
        """
        timestamps = sorted(self.states.keys())
        pitch = [self.states[timestamp][3] for timestamp in timestamps]
        yaw = [self.states[timestamp][4] for timestamp in timestamps]
        x = [self.states[timestamp][0] for timestamp in timestamps]
        y = [self.states[timestamp][1] for timestamp in timestamps]
        z = [self.states[timestamp][2] for timestamp in timestamps]
        
        pitch_rate = [self.states[timestamp][8] for timestamp in timestamps]
        yaw_rate = [self.states[timestamp][9] for timestamp in timestamps]
        vx = [self.states[timestamp][5] for timestamp in timestamps]
        vy = [self.states[timestamp][6] for timestamp in timestamps]
        vz = [self.states[timestamp][7] for timestamp in timestamps]
        
        steering_angle = [self.states[timestamp][10] for timestamp in timestamps]
        slipping_angle = [self.states[timestamp][11] for timestamp in timestamps]
        
        for time in self.abnormal_timestamps:
            if time not in timestamps:
               print("no time:  ", time)
        
        vis_traj({'pitch': pitch, 'yaw': yaw, 'x': x, 'y': y, 'z': z}, join(vis_dir, 'pose.png'), timestamps, self.abnormal_timestamps)
        
        vis_traj({'pitch_rate': pitch_rate, 'yaw_rate': yaw_rate, 'vx': vx, 'vy': vy, 'vz': vz}, join(vis_dir, 'state_in_vcs.png'), timestamps, self.abnormal_timestamps)
        
        vis_traj({'steering_angle': steering_angle, 'slipping_angle': slipping_angle}, join(vis_dir, 'steer_slip_angle.png'), timestamps, self.abnormal_timestamps)


    def detect_abnormal(self, pose_tum_array=None):
        """detect abnormal poses

        Args:
            pose_tum_array (ndarray, optional): input pose in tum array (must be vcs coordinate!). Defaults to None.

        Returns:
            list: list of abnormal timestamps in seconds.
        """
        if pose_tum_array is not None:
            self.load_pose(pose_tum_array)
        timestamps = self.pose_tum_array[:, 0]
        poses = self.pose_tum_array[:, 1:]
        self.abnormal_timestamps = []
        self.states = {}

        for i in range(len(timestamps)-1):
            dt = timestamps[i+1] - timestamps[i]

            cur_2_next = self.relative_transforms[i]
            next_2_cur = invT(cur_2_next)
            x, y, z = poses[i, :3]
            yaw, pitch, roll = R.from_quat(poses[i, 3:]).as_euler('zyx')
            delta_yaw, delta_pitch, delta_roll = R.from_matrix(next_2_cur[:3, :3]).as_euler('zyx')
            cur_yaw_rate = delta_yaw / dt
            cur_pitch_rate = delta_pitch / dt

            # current vx, vy, yaw, yawrate
            cur_vx = next_2_cur[0, 3] / dt
            cur_vy = next_2_cur[1, 3] / dt
            cur_vz = next_2_cur[2, 3] / dt
            
            if cur_vx < self.static_threshold:
                steering_angle = 0.
                slipping_angle = 0.
            else:
                slipping_angle = np.arctan2(cur_vy, cur_vx)
                steering_angle = np.arctan2(self.wheelbase * cur_yaw_rate, cur_vx * np.cos(slipping_angle))

            steering_angle = steering_angle * 180. / np.pi
            if np.abs(steering_angle) > self.steering_threshold:
                self.abnormal_timestamps.append(timestamps[i])
            
            slipping_angle = slipping_angle * 180. / np.pi
            if np.abs(slipping_angle) > self.slipping_threshold:
                self.abnormal_timestamps.append(timestamps[i])

            cur_yaw_rate = cur_yaw_rate * 180. / np.pi
            cur_pitch_rate = cur_pitch_rate * 180. / np.pi
            if np.abs(cur_pitch_rate) > self.pitch_rate_threshold:
                self.abnormal_timestamps.append(timestamps[i])
                
            if np.abs(cur_vz) > self.vertical_vel_threshold:  # or
                self.abnormal_timestamps.append(timestamps[i])

            self.states[timestamps[i]] = [x, y, z, pitch * 180. / np.pi, yaw * 180. / np.pi,
                                          cur_vx, cur_vy, cur_vz,
                                          cur_pitch_rate, cur_yaw_rate,
                                          steering_angle, slipping_angle]

        return self.abnormal_timestamps

if __name__ == '__main__':
    odometry_path = "test/sample_clip/4GD36_20220502_103454/odometry/vehicle_kinematic_example.txt"
    tum_array = np.loadtxt(odometry_path)

    vis_dir = "tmp"
    vk = VehicleKinematic(tum_array)
    abonormals = vk.detect_abnormal()
    vk.plot_trajectory(vis_dir)

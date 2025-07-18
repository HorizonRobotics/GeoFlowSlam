import os
import cv2
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from cv_bridge import CvBridge
import numpy as np
import argparse
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.state = None

    def filter(self, value):
        if self.state is None:
            self.state = value
        else:
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state
def extract_images_and_imu(bag_file, color_dir, depth_dir, imu_dir):
    # 创建文件夹
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    bridge = CvBridge()
    color_num = 0
    depth_num = 0
    imu_num = 0
    odom_num=0
    imu_data=[]
    with Reader(bag_file) as reader, open(os.path.join(imu_dir, "imu.txt"), 'w') as imu_f, open(os.path.join(imu_dir, "odom.txt"), 'w') as odom_f:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            try:
                msg = deserialize_cdr(rawdata, connection.msgtype)
            except Exception as e:
                # print(f"Skipping message on topic {topic} due to error: {e}")
                continue
            # topic = connection.topic

            # 从消息中获取时间戳
            # print(msg.header.stamp)
            # ms
            timestamp = float(msg.header.stamp.sec)*1e3+ float(msg.header.stamp.nanosec)*1e-6
            # timestamp=msg.header.stamp.to_sec()*1e3
            # print("timestamp: ", timestamp)

            if topic == '/camera/color/image_raw':
                # 提取彩色图像
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image_filename = os.path.join(color_dir, f"{timestamp:.4f}.png")
                cv2.imwrite(image_filename, cv_image)
                # print(msg.header.stamp)
                print("timestamp: ", timestamp)
                color_num += 1

            elif topic == '/camera/aligned_depth_to_color/image_raw':
                # 提取深度图像
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_filename = os.path.join(depth_dir, f"{timestamp:.4f}.png")
                cv2.imwrite(depth_filename, cv_image)
                depth_num += 1

            elif topic == '/camera/imu':
                # 提取IMU数据
                accel = msg.linear_acceleration
                gyro = msg.angular_velocity
                imu_f.write(f"{timestamp:.4f},{accel.x},{accel.y},{accel.z},{gyro.x},{gyro.y},{gyro.z}\n")
                imu_num += 1
            elif topic =='/robot_odom':
                # 提取IMU数据twist
                # print(msg.twist.twist)
                twist=msg.twist.twist
                linear=twist.linear
                angular=twist.angular
                odom_f.write(f"{timestamp:.4f},{linear.x},{linear.y},{linear.z},{angular.x},{angular.y},{angular.z}\n")
                odom_num+=1
                # pos = msg.pose.pose.position
                # quaternion = msg.pose.pose.orientation
                # odom_f.write(f"{timestamp:.4f} {pos.x} {pos.y} {pos.z} {quaternion.x} {quaternion.y} {quaternion.z} {quaternion.w}\n")
                # odom_num += 1
    print(f"Extracted {color_num} color images, {depth_num} depth images, and {imu_num} IMU data.")

if __name__ == "__main__":
    #/home/tingyang/3d_recon/dataset/intel/2025-03-10-17-04-54
    # bag_file = "/home/users/tingyang.xiao/3D_Recon/datasets/intel/2025-03-10-17-04-54"
    # color_dir = "/home/users/tingyang.xiao/3D_Recon/datasets/intel/2025-03-10-17-04-54/color"
    # depth_dir = "/home/users/tingyang.xiao/3D_Recon/datasets/intel/2025-03-10-17-04-54/depth"
    # imu_dir = "/home/users/tingyang.xiao/3D_Recon/datasets/intel/2025-03-10-17-04-54/imu"

    # extract_images_and_imu(bag_file, color_dir, depth_dir, imu_dir)
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_root", required=True, help="Path to the data root")
    parser.add_argument("--data_list", required=True, nargs='+', help="List of data directories")
    args = parser.parse_args()
    # 遍data_list与data_root拼接
    data_root = args.data_root
    data_list = args.data_list
    for data_dir in data_list:
        bag_file = os.path.join(data_root, data_dir)
        color_dir = os.path.join(bag_file, "color")
        depth_dir = os.path.join(bag_file, "depth")
        imu_dir = os.path.join(bag_file, "imu")

        extract_images_and_imu(bag_file, color_dir, depth_dir, imu_dir)
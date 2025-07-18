import matplotlib.pyplot as plt

data_root = "/home/tingyang/3d_recon/dataset/unitree_legged_robotic_datasets/g1/"
datalist = [
    "rosbag2_2025_07_07-16_28_22/imu/imu.txt",
    "rosbag2_2025_07_14-15_14_55/imu/imu.txt",

]
# datalist = [
#     "rosbag2_2024_12_24-18_41_50/imu/imu.txt",
#     "rosbag2_2024_12_06-10_35_09/imu/imu.txt",
#     "rosbag2_1970_01_04-04_36_13/imu/imu.txt",
#     "rosbag2_1970_01_04-04_45_37/imu/imu.txt"
# ]

plt.style.use('seaborn-v0_8-paper')  # 学术风格

labels = ['Acc X', 'Acc Y', 'Acc Z']
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
# colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']  # 鲜艳的蓝、橙、绿、红
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#FFB6C1']  # 蓝、橙、绿、粉红
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#DC143C']  # 蓝、橙、绿、红
legend_names = [
    "Seq 5", "Seq 6"
]

# legend_names = [
#     "Seq 1", "Seq 2", "Seq 3", "Seq 4"
# ]
# 可根据实际数据范围调整
y_limits = [(-25, 25), (-25, 25), (-20, 20)]  # 示例：分别为X、Y、Z轴加速度的y轴范围
x_limit = (0, 450)  # 示例：横坐标时间范围（秒）
fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

for idx, data_file in enumerate(datalist):
    acc_x, acc_y, acc_z, timestamps = [], [], [], []
    with open(data_root + data_file, "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 7:
                continue
            timestamps.append(float(parts[0]) * 1e-3)  # ms转s
            acc_x.append(float(parts[1]))
            acc_y.append(float(parts[2]))
            acc_z.append(float(parts[3]))
    t0 = timestamps[0]
    timestamps = [t - t0 for t in timestamps]
    axs[0].scatter(timestamps, acc_x, label=legend_names[idx], color=colors[idx], s=6, alpha=0.7)
    axs[1].scatter(timestamps, acc_y, label=legend_names[idx], color=colors[idx], s=6, alpha=0.7)
    axs[2].scatter(timestamps, acc_z, label=legend_names[idx], color=colors[idx], s=6, alpha=0.7)

for i in range(3):
    axs[i].set_ylabel(f'{labels[i]} (m/s²)', fontsize=12)
    axs[i].tick_params(axis='y', labelsize=14)
    axs[i].tick_params(axis='x', labelsize=14)
    axs[i].grid(True, linestyle='--', alpha=0.6)
    axs[i].legend(fontsize=15, loc='upper right',markerscale=2)
    axs[i].set_ylim(y_limits[i])  # 设置y轴范围
    axs[i].set_xlim(x_limit)      # 设置x轴范围
axs[2].set_xlabel('Time (s)', fontsize=16)
fig.suptitle('IMU Acceleration Comparison (All Sequences)', fontsize=14, fontweight='bold')
plt.tight_layout(rect=[0, 0, 1, 0.97])
# plt.show()
save_path = "/home/tingyang/3d_recon/geoflow-slam/script/evaluator/ours/imu_seq.png"
plt.savefig(save_path, dpi=600, bbox_inches='tight')
print(f"Figure saved to {save_path}")
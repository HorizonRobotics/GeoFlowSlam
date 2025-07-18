# pip install rosbags
# 首先解析原始数据
##指定解析的数据路径
data_root="/home/tingyang/3d_recon/dataset/G1/"
# data_list=("rosbag2_2025_07_04-11_38_27")
data_root="/home/tingyang/3d_recon/dataset/unitree_legged_robotic_datasets/g1/"
data_list=("rosbag2_2025_07_14-15_14_55")


echo "Parse data********"
python3 /home/tingyang/3d_recon/geoflow-slam/script/tools/parse_ros2_bag.py --data_root $data_root --data_list $data_list
echo "Parse data done********"



scene=(long_office_household nostructure_notexture_far
 nostructure_notexture_near_withloop nostructure_texture_far 
 nostructure_texture_near_withloop structure_notexture_far 
 structure_notexture_near structure_notexture_far 
 structure_texture_far structure_texture_near)
scene=(nostructure_notexture_far nostructure_notexture_near_withloop )
scene=(large_cabinet)
echo "Start to evaluate the new NoOF_NoICP_Lidar version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory_NoOF_NoICP_Lidar.txt \
        --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
        --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
        --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_NoOF_NoICP_Lidar.json
    done
echo "Pose evaluation is new NoOF_NoICP_Lidar version done..."
echo "Start to evaluate the new OF_ICP_Lidar version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory_OF_ICP_Lidar.txt \
        --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
        --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
        --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_ICP_Lidar.json
    done
echo "Pose evaluation is new OF_ICP_Lidar version done..."
echo "Start to evaluate the new OF_ICP version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory_OF_ICP.txt \
        --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
        --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
        --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_ICP.json
    done
echo "Pose evaluation is new NoOF_ICP version done..."

echo "Start to evaluate the new NoOF_ICP version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory_NoOF_ICP.txt \
        --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
        --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
        --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_NoOF_ICP.json
    done
echo "Pose evaluation is new NoOF_ICP version done..."

echo "Start to evaluate the new OF_NoICP version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory_OF_NoICP.txt \
        --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
        --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
        --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_NoICP.json
    done
echo "Pose evaluation is new OF_NoICP version done..."



# echo "Start to evaluate the new baseline version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/CameraTrajectory.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --association /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/associate.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_baseline.json
#     done
# echo "Pose evaluation is new baseline version done..."
# echo "Start to evaluate the new NoOF_NoICP_Lidar version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/KeyFrameTrajectory_NoOF_NoICP_Lidar.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_NoOF_NoICP_Lidar.json
#     done
# echo "Pose evaluation is new NoOF_NoICP_Lidar version done..."
# echo "Start to evaluate the new OF_ICP_Lidar version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/KeyFrameTrajectory_OF_ICP_Lidar.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_ICP_Lidar.json
#     done
# echo "Pose evaluation is new OF_ICP_Lidar version done..."
# echo "Start to evaluate the new OF_ICP version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/KeyFrameTrajectory_OF_ICP.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_ICP.json
#     done
# echo "Pose evaluation is new NoOF_ICP version done..."

# echo "Start to evaluate the new NoOF_ICP version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/KeyFrameTrajectory_NoOF_ICP.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_NoOF_ICP.json
#     done
# echo "Pose evaluation is new NoOF_ICP version done..."

# echo "Start to evaluate the new OF_NoICP version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluatorTUM.py --pred /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/KeyFrameTrajectory_OF_NoICP.txt \
#         --gt /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/groundtruth.txt \
#         --result /home/tingyang/3d_recon/dataset/tum_dataset/rgbd_dataset_freiburg3_${i}/rgbd_dataset_freiburg3_${i}_OF_NoICP.json
#     done
# echo "Pose evaluation is new OF_NoICP version done..."
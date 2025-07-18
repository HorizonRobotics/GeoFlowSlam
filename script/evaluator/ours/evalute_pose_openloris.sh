
# scene=(1 2 3 4 5 6 7)
# echo "Start to evaluate the new OF_ICP version pose..."
#     for i in ${scene[@]}; do
#         python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/dataset/openloris/office/office1-${i}/CameraTrajectory_OF_ICP_Lidar.txt \
#         --gt /home/tingyang/3d_recon/dataset/openloris/office/office1-${i}/gt/gt.txt \
#         --result /home/tingyang/3d_recon/dataset/openloris/office/office1-${i}/CameraTrajectory_OF_ICP.json
#     done
# echo "Pose evaluation is new OF_ICP version done..."

scene=(1)
echo "Start to evaluate the new OF_ICP version pose..."
    for i in ${scene[@]}; do
        python3 PoseEvaluator.py --pred /home/tingyang/3d_recon/dataset/openloris/home/home1-${i}/CameraTrajectory_OF_ICP_Lidar.txt \
        --gt /home/tingyang/3d_recon/dataset/openloris/home/home1-${i}/gt/gt.txt \
        --result /home/tingyang/3d_recon/dataset/openloris/home/home1-${i}/CameraTrajectory_OF_ICP.json
    done
echo "Pose evaluation is new OF_ICP version done..."
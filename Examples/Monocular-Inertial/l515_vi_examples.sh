#!/bin/bash
pathDatasetTUM_VI='/root/horizon/3D_Recon/dataset/L515/20240929_105625' #Example, it is necesary to change it by the dataset path

#------------------------------------

# #------------------------------------
# Monocular-Inertial Examples
echo "Launching Corridor 1 with Monocular-Inertial sensor"
# ./mono_inertial ../../Vocabulary/ORBvoc.txt ./L515-VI.yaml \
#  /home/users/tingyang.xiao/3D_Recon/datasets/dataset_L515/2024-10-16-16-21-57\
#  /home/users/tingyang.xiao/3D_Recon/datasets/dataset_L515/2024-10-16-16-21-57/associate.txt

  ./mono_inertial ../../Vocabulary/ORBvoc.txt ./L515-VI.yaml \
 /root/horizon/3D_Recon/dataset/L515/record_by_ros/2024-10-16-16-21-57 \
 /root/horizon/3D_Recon/dataset/L515/record_by_ros/2024-10-16-16-21-57/associate.txt
# ./mono_inertial ../../Vocabulary/ORBvoc.txt ./openloris.yaml \
#  /root/horizon/3D_Recon/dataset/OpenLORIS/cafe1-2 \
#  /root/horizon/3D_Recon/dataset/OpenLORIS/cafe1-2/associate.txt
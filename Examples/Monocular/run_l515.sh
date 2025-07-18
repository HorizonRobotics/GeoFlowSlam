#!/bin/bash
pathDatasetTUM_VI='/root/horizon/3D_Recon/dataset/L515/20240929_105625' #Example, it is necesary to change it by the dataset path

#------------------------------------

# #------------------------------------
# Monocular-Inertial Examples
echo "Launching Corridor 1 with Monocular sensor"
# ./mono_inertial_l515 ../../Vocabulary/ORBvoc.txt ./L515-VI.yaml \
#  /root/horizon/3D_Recon/dataset/L515/20240926_155347\
#  /root/horizon/3D_Recon/dataset/L515/20240926_155347/associate.txt

./mono_l515 ../../Vocabulary/ORBvoc.txt ./openloris.yaml \
 /root/horizon/3D_Recon/dataset/OpenLORIS/cafe1-2 \
 /root/horizon/3D_Recon/dataset/OpenLORIS/cafe1-2/associate.txt
echo "Building ROS nodes"

#!/bin/bash
set -e
TOOL_CHAIN_CMD=
ENABLE_VIEWER=ON
ENABLE_ASYNC=ON
ENABLE_OMP=ON
FILE_PATH=`pwd`

clean_Thirdparty() {
  echo "clean Thirdparty"
  cd ${FILE_PATH}/Thirdparty
  for file in ./*
  do
    if [ -d "${file}" ]
    then
      echo "clean the ${file}"
      rm -rf ${file}/build ${file}/lib
    fi
  done
}

build_Thirdparty() {
  echo "build Thirdparty"
  cd ${FILE_PATH}/Thirdparty
  for file in ./*
  do
    if [ -d "${file}" ]
    then
      echo "build the ${file}"
      cd ${file}
      rm -rf ./build ./lib
      mkdir build
      cd build
      cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD}
      make -j
      if [ -d "../lib" ]
      then
        cp ../lib/* ../../../ROS2/RGB-D-Inertial/lib/
      fi
      cd ../..
    fi
  done
}

build_Examples_ROS2(){
  echo "build Examples_ROS2"
  cd ${FILE_PATH}/Examples_ROS2
  colcon build --merge-install  --cmake-force-configure  --cmake-args  -DCMAKE_BUILD_TYPE=Release  -DBUILD_TESTING:BOOL=OFF  ${TOOL_CHAIN_CMD}
  cd ..
  
}


uncompress_vocabulary() {
  echo "Uncompress vocabulary ..."
  cd ${FILE_PATH}/Vocabulary
  tar -xvf ORBvoc.txt.tar.gz
  cd ..
}



# build_Thirdparty
cd Examples/ROS2/RGB-D-Inertial
# rm -rf build
# rm -rf install
# rm -rf log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=${ENABLE_VIEWER} -DENABLE_ASYNC=${ENABLE_ASYNC} -DENABLE_OMP=${ENABLE_OMP} ${TOOL_CHAIN_CMD}

#build_Examples_ROS2
#clean_Thirdparty
# uncompress_vocabulary


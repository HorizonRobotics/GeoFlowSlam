#!/bin/bash
set -e
TOOL_CHAIN_CMD=
ENABLE_VIEWER=ON
ENABLE_ASYNC=ON
REGISTER_TIMES=OFF
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
      cmake .. -DCMAKE_BUILD_TYPE=Release
      make -j
      # if [ -d "../lib" ]
      # then
      #   cp ../lib/* ../../lib/
      # fi
      cd ../..
    fi
  done
}


uncompress_vocabulary() {
  echo "Uncompress vocabulary ..."
  cd ${FILE_PATH}/Vocabulary
  tar -xvf ORBvoc.txt.tar.gz
  cd ..
}


# clean_Thirdparty
# build_Thirdparty
# uncompress_vocabulary
# echo "Configuring and building ORB_SLAM3 ..."
# cd ${FILE_PATH}
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=${ENABLE_VIEWER} \
 -DENABLE_ASYNC=${ENABLE_ASYNC} -DENABLE_OMP=${ENABLE_OMP} -DREGISTER_TIMES=${REGISTER_TIMES} ${TOOL_CHAIN_CMD}
make -j4

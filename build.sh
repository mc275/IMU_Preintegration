#!/bin/sh

cd g2o/

echo "Configuring and building Thirdparty/g2o ..."
cd Thirdparty/g2o
if [ ! -d "build/" ];then
mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../
echo "Configuring and building IMU_Preintegration ..."
if [ ! -d "/build" ];then
mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ..

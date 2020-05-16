brew install eigen vtk tbb

cd thirdparty/opencv
rm -r build
mkdir build&cd build
cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_ENABLE_NONFREE:BOOL=ON \
 -DWITH_VTK=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.10/modules ..
make -j8
make install


cd thirdparty/Sophus
rm -r build
mkdir build&cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../g2o
rm -r build
mkdir build&cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8


brew install brewsci/science/g2o
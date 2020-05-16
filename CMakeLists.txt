cmake_minimum_required(VERSION 3.15)
project(TinySLAM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -std=c++17")
# Set third party lib
#set(OpenCV_DIR /usr/local/opt/opencv@3/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package( G2O REQUIRED)
set(OpenCV_STATIC ON)

# Set include
include_directories( include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})
# Set dylib
link_directories( ${OpenCV_LIB_DIRS} )
# Set source file
AUX_SOURCE_DIRECTORY(src LIB_DIR)

# Build toySLAM library
SET(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)
add_library(TinySLAM SHARED ${LIB_DIR})
target_link_libraries( TinySLAM
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        /usr/local/lib/libg2o_core.dylib
        /usr/local/lib/libg2o_stuff.dylib
        /usr/local/lib/libg2o_types_sba.dylib
        )

# Build examples
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(Euroc test/Euroc.cpp)
target_link_libraries(Euroc TinySLAM)

add_executable(zju test/zju.cpp)
target_link_libraries(zju TinySLAM)

add_executable(TUM test/TUM.cpp)
target_link_libraries(TUM TinySLAM)


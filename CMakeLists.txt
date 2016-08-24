cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1)
project( visodom )

find_package( OpenCV REQUIRED )
find_package( GTest REQUIRED )
include_directories(${GTEST_INCLUDE_DIRS})

set(OBJ_FILES
        Pose.h Pose.cc
        PointCloud.cc PointCloud.h
        MatUtils.cc MatUtils.h
        Tracking.cc Tracking.h Mapping.cc Mapping.h)
set(TEST_FILES test/PoseTest.cc test/PointCloudTest.cc test/TestHeader.h test/TrackingTest.cc test/MappingTest.cc)

add_executable( visodom imshow.cc )
target_link_libraries( visodom ${OpenCV_LIBS} )

add_executable( tests ${TEST_FILES} ${OBJ_FILES})
target_link_libraries( tests ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
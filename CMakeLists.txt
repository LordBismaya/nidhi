cmake_minimum_required(VERSION 2.8.3)
project(nidhi)
#add_definitions(-Wall -std=c++11 -Werror=return-type ${PCL_DEFINITIONS})
add_definitions(-Wcpp ${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf pcl_ros)


set(CMAKE_BUILD_TYPE Debug)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf)

set(DENSE_SOURCE_FILES     
		src/nidhi_mapper.cpp
#                src/DenseMapping.cpp
                src/nidhi_tracker.cpp
                src/nidhi_system.cpp
		src/nidhi_datastruct.cpp
#                src/superpixel.cpp
		src/main.cpp
)


set(DENSE_HEADER_FILES  
		include/nidhi/nidhi_mapper.h
#                include/dpptam/DenseMapping.h
                include/nidhi/nidhi_tracker.h
                include/nidhi/nidhi_system.h
                include/nidhi/nidhi_headers.h
		include/nidhi/nidhi_datastruct.h
#                include/dpptam/superpixel.h
)

find_package(OpenCV REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  LIBRARIES imu_3dm_gx3
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
#find_package(OpenMP)
#if(OPENMP_FOUND)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()



#option(OPTIMIZATION "Enable optimization" TRUE)
#if(OPTIMIZATION)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
#message("OPTIMIZATION FLAG ACTIVATED")
#else()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
#message("NO OPTIMIZATION FLAGS")
#endif()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")

link_directories(${PCL_LIBRARY_DIRS})
###########
## Build ##
###########
add_executable(nidhi ${DENSE_HEADER_FILES} ${DENSE_SOURCE_FILES})
target_link_libraries(nidhi ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)


 install(PROGRAMS
  scripts/run_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

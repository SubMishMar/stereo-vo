cmake_minimum_required( VERSION 2.8 )
project( stereo-vo )
find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io visualization)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
add_executable( stereo_vo
		src/svo.cpp
		src/paramreader.cpp
		src/init.cpp
		src/featureDetectionAndTracking.cpp
		src/drawers.cpp
		src/filter.cpp)
target_link_libraries(stereo_vo ${OpenCV_LIBS} #${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
					  #${PCL_VISUALIZATION_LIBRARIES}
					   )

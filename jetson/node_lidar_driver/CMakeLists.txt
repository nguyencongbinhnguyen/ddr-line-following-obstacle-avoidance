# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 2.8)

project(ydlidar_ros_driver)

############## YDLIDAR SDK START#####################################
# Tìm gói ydlidar_sdk
find_package(ydlidar_sdk REQUIRED)
############## YDLIDAR SDK END#####################################

## Tìm các thư viện catkin cần thiết
find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    sensor_msgs
    visualization_msgs    # THÊM MỚI: Để vẽ Marker vùng an toàn [cite: 131, 141]
    geometry_msgs         # THÊM MỚI: Để định nghĩa tọa độ điểm P, Q [cite: 169, 171]
    serial
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS
    roscpp 
    rospy 
    sensor_msgs 
    visualization_msgs 
    geometry_msgs 
    message_runtime
)

#---------------------------------------------------------------------------------------
# Cấu hình Build và Compiler
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Thư mục Include
include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${YDLIDAR_SDK_INCLUDE_DIRS}
)

# Thư mục Link thư viện
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

#---------------------------------------------------------------------------------------
# Tạo file thực thi (Executable)
#---------------------------------------------------------------------------------------
# Sử dụng file ydlidar_ros_driver_node.cpp mà bạn vừa lưu kết hợp [cite: 157, 173]
#add_executable(${PROJECT_NAME}_node src/ydlidar_ros_driver_node.cpp)
#add_executable(serial_test_node src/serial_test_node.cpp)
add_executable(${PROJECT_NAME}_node src/ydlidar_ros.cpp)

#---------------------------------------------------------------------------------------
# Liên kết thư viện (Link libraries)
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${YDLIDAR_SDK_LIBRARIES}
)
#target_link_libraries(serial_test_node
#  ${catkin_LIBRARIES}
#)
#---------------------------------------------------------------------------------------
# Cài đặt (Install)
#---------------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





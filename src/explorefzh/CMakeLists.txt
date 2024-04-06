cmake_minimum_required(VERSION 3.0.2)
project(explorefzh)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS 
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    
    std_msgs
    tf 
    visualization_msgs

)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src  SRC_LIST)

add_executable(${PROJECT_NAME}_node  ${SRC_LIST})




# add_executable(${PROJECT_NAME}_node src/active_explore.cpp src/costmap_client.cpp  src/search_frontier.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)
endif()


# 这部分代码块主要涉及到ROS软件包的安装和测试相关的配置。让我们逐个解释其作用：

# install(TARGETS explore ...)：

# 这个命令用于安装目标文件，即构建生成的可执行文件或库文件。
# TARGETS explore指定了要安装的目标是名为explore的目标，这通常是通过add_executable()或add_library()命令定义的。
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}指定了安装目标的归档文件（静态库）应该安装到的目录。
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}指定了安装目标的库文件（动态库）应该安装到的目录。
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}指定了安装目标的可执行文件应该安装到的目录。
# 这样配置后，当执行catkin_make install命令时，会将构建生成的目标文件安装到对应的目录下。
# install(DIRECTORY launch/ ...)：

# 这个命令用于安装目录下的文件或目录。
# DIRECTORY launch/指定了要安装的目录是名为launch的目录，其中包含了一些启动文件（launch文件）。
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch指定了安装目录的目标路径，这里是将launch/目录安装到${CATKIN_PACKAGE_SHARE_DESTINATION}/launch目录下。
# 这样配置后，启动文件会被安装到ROS软件包的共享目录下，其他ROS节点可以通过roslaunch命令启动这些文件。
# if(CATKIN_ENABLE_TESTING)和roslaunch_add_file_check(launch)：

# 这部分代码用于配置ROS软件包的测试功能。
# if(CATKIN_ENABLE_TESTING)用于检查是否启用了测试功能，如果启用了则执行下面的测试相关配置，否则跳过。
# find_package(roslaunch REQUIRED)用于查找ROS的roslaunch包，这是用于启动ROS启动文件的工具。
# roslaunch_add_file_check(launch)用于指定测试目标，这里指定了测试所有位于launch/目录下的启动文件。
# 这样配置后，在执行catkin_make run_tests命令时，会运行测试启动文件，用于检查ROS节点的启动行为和结果是否符合预期。
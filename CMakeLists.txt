cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
)

###########
## Build ##
###########

### Utils Library
add_library(utils_api
  src/utils.cpp
)

target_include_directories(utils_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(utils_api
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(utils_api
  ${catkin_EXPORTED_TARGETS}
)

### Kalman Filter Library


## KF - Base Class

add_library(kf_api
  src/kf.cpp
)

target_include_directories(kf_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(kf_api
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(kf_api
  ${catkin_EXPORTED_TARGETS}
)

## EKF - EKF Class

add_library(ekf_api
  src/ekf.cpp
)

target_include_directories(ekf_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(ekf_api
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(ekf_api
  ${catkin_EXPORTED_TARGETS}
)


## UKF - UKF Class

add_library(ukf_api
  src/ukf.cpp
)

target_include_directories(ukf_api
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(ukf_api
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(ukf_api
  ${catkin_EXPORTED_TARGETS}
)

#############
## Testing ##
#############

################
# Utils
################
catkin_add_gtest(utils_test test/utils_test.cpp)

target_include_directories(utils_test
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(utils_test utils_api
                      ${EIGEN3_LIBRARIES}
                      ${catkin_LIBRARIES})

add_dependencies(utils_test
  ${catkin_EXPORTED_TARGETS})

################
# Kalman Filter
################
catkin_add_gtest(kf_test test/kf_test.cpp 
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/test/data/")

target_include_directories(kf_test
  SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(kf_test kf_api ekf_api ukf_api utils_api
                      ${EIGEN3_LIBRARIES}
                      ${catkin_LIBRARIES})

add_dependencies(kf_test
  ${catkin_EXPORTED_TARGETS})
cmake_minimum_required(VERSION 2.8.3)
project(dr_eigen)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	dr_param
	geometry_msgs
	cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES dr_eigen ${Eigen_LIBRARIES}
	CATKIN_DEPENDS roscpp dr_param geometry_msgs
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/param.cpp
	src/yaml.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
)

dr_add_gtest(average           test/average.cpp)
dr_add_gtest(axes              test/axes.cpp)
dr_add_gtest(box               test/box.cpp)
dr_add_gtest(compare           test/compare.cpp)
dr_add_gtest(plane             test/plane.cpp)
dr_add_gtest(translate         test/translate.cpp)
dr_add_gtest(rotate            test/rotate.cpp)
dr_add_gtest(ros_to_eigen      test/ros_to_eigen.cpp)
dr_add_gtest(eigen_to_ros      test/eigen_to_ros.cpp)
dr_add_gtest(tf_to_eigen       test/tf_to_eigen.cpp)
dr_add_gtest(eigen_to_tf       test/eigen_to_tf.cpp)
dr_add_gtest(param_vector      test/param_vector.cpp)
dr_add_gtest(param_quaternion  test/param_quaternion.cpp)
dr_add_gtest(param_isometry    test/param_isometry.cpp)
dr_add_gtest(yaml              test/yaml.cpp)

target_link_libraries(${PROJECT_NAME}_test_param_vector     ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_param_quaternion ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_param_isometry   ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_eigen_to_tf      ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_tf_to_eigen      ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_yaml             ${PROJECT_NAME})

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

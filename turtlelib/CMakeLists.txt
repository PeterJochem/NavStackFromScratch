cmake_minimum_required(VERSION 3.9)
project(turtlelib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Create the turtlelib library target
add_library(turtlelib src/rigid2d.cpp)
target_link_libraries(turtlelib Eigen3::Eigen)
#target_include_directories(turtlelib INTERFACE include/)
target_compile_features(turtlelib PUBLIC cxx_std_17)
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

add_executable(frame_main_executable src/frame_main.cpp)
target_link_libraries(frame_main_executable turtlelib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(${PROJECT_NAME}_test_executable tests/tests.cpp)
  target_include_directories(${PROJECT_NAME}_test_executable PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  #ament_target_dependencies(${PROJECT_NAME} std_msgs)
  target_link_libraries(${PROJECT_NAME}_test_executable turtlelib)
endif()

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/turtlelib DESTINATION include)
install(TARGETS turtlelib EXPORT turtlelib-targets)

install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/${PROJECT_NAME})

ament_package()

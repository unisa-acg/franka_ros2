cmake_minimum_required(VERSION 3.14)
project(franka_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math -ftree-vectorize")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ffast-math -ftree-vectorize -DNDEBUG -DEIGEN_NO_DEBUG")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
 set(EIGEN_VECTORIZE TRUE)      # enables vectorization
 set(EIGEN_NO_DEBUG TRUE)       # disables Eigen assertions for speed
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka 0.9.3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(acg_optimal_control REQUIRED)

add_library(franka_hardware SHARED
        src/franka_action_server.cpp
        src/franka_hardware_interface.cpp
        src/franka_async_hardware_interface.cpp
        src/robot_communication_thread.cpp
        src/robot.cpp
        src/franka_param_service_server.cpp
        src/franka_executor.cpp
        src/panda_joint_command_governor.cpp
)

target_include_directories(
        franka_hardware
        PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${acg_optimal_control_INCLUDE_DIRS}
)

target_link_libraries(
        franka_hardware
        PUBLIC
        acg_optimal_control::acg_optimal_control
)

ament_target_dependencies(
        franka_hardware
        PUBLIC
        ament_index_cpp
        hardware_interface
        Franka
        pluginlib
        rclcpp
        rclcpp_action
        franka_msgs
        realtime_tools
)

target_compile_definitions(franka_hardware PRIVATE
    PACKAGE_NAME="${PROJECT_NAME}"
)

pluginlib_export_plugin_description_file(hardware_interface franka_hardware.xml)

install(
        TARGETS franka_hardware
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}/data
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    add_subdirectory(test)

    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)

ament_export_libraries(
        franka_hardware
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()

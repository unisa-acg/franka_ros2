cmake_minimum_required(VERSION 3.5)
project(franka_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

## LTTng-UST instrumentation (franka_timing tracepoint provider, see tracing/tp.h)
option(WITH_LTTNG "Instrument the FCI loop with LTTng-UST tracepoints (needs liblttng-ust-dev >= 2.13)" ON)
set(FRANKA_LTTNG_ENABLED FALSE)
if(WITH_LTTNG)
  find_path(LTTNG_UST_INCLUDE_DIR NAMES lttng/tracepoint.h)
  find_library(LTTNG_UST_LIBRARY NAMES lttng-ust)
  if(LTTNG_UST_INCLUDE_DIR AND LTTNG_UST_LIBRARY)
    set(FRANKA_LTTNG_ENABLED TRUE)
  else()
    message(WARNING "WITH_LTTNG is ON but liblttng-ust-dev (>= 2.13) was not found; "
                    "building without FCI loop instrumentation. Install liblttng-ust-dev "
                    "or pass -DWITH_LTTNG=OFF to silence this warning.")
  endif()
endif()

if(FRANKA_LTTNG_ENABLED)
  add_library(franka_hardware_tracing STATIC tracing/tp.c)
  target_include_directories(franka_hardware_tracing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tracing>
  )
  target_include_directories(franka_hardware_tracing SYSTEM PUBLIC ${LTTNG_UST_INCLUDE_DIR})
  target_link_libraries(franka_hardware_tracing PUBLIC ${LTTNG_UST_LIBRARY})
  target_compile_definitions(franka_hardware_tracing PUBLIC FRANKA_WITH_LTTNG_UST)
else()
  add_library(franka_hardware_tracing INTERFACE)
  target_include_directories(franka_hardware_tracing INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tracing>
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka 0.9.3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(franka_hardware SHARED
        src/franka_action_server.cpp
        src/franka_hardware_interface.cpp
        src/franka_async_hardware_interface.cpp
        src/robot_communication_thread.cpp
        src/robot.cpp
        src/franka_param_service_server.cpp
        src/franka_executor.cpp
)

target_include_directories(
        franka_hardware
        PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        franka_hardware
        hardware_interface
        Franka
        pluginlib
        rclcpp
        rclcpp_action
        franka_msgs
        realtime_tools
)

target_link_libraries(
        franka_hardware
        PRIVATE
        franka_hardware_tracing
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

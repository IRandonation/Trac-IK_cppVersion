cmake_minimum_required(VERSION 3.10)
project(TRAC_IK VERSION 1.0.0 LANGUAGES CXX)

set(BUILD_SHARED_LIBS OFF)

# ---------- 编译环境 ----------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 设置MinGW编译器
if(MINGW)
    set(CMAKE_CXX_COMPILER g++)
    set(CMAKE_C_COMPILER gcc)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# ---------- 目录 ----------
set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${ROOT_DIR}/include)

# ---------- NLopt 路径 ----------
# 1. 头文件
set(NLOPT_HPP_DIR "${ROOT_DIR}/third_party/nlopt/build")
set(NLOPT_H_DIR   "${ROOT_DIR}/third_party/nlopt/src/api")
# 2. 库文件
set(NLOPT_LIB_DIR     "${ROOT_DIR}/third_party/nlopt/build/")
# 3. 库名
set(NLOPT_LIB         nlopt)

# ---------- orocos_kdl (KDL) 路径 ----------
set(KDL_ROOT_DIR "D:/Project/JD_Robot/RobotArm/TRAC-IK/Trac-ik-cpp/third_party/orocos_kdl")
set(KDL_INCLUDE_DIR "${KDL_ROOT_DIR}/src")
set(KDL_LIB_DIR "${KDL_ROOT_DIR}/build/src")
set(KDL_LIB orocos-kdl)

# ---------- Eigen 路径 ----------
set(EIGEN_ROOT_DIR "D:/Project/JD_Robot/RobotArm/TRAC-IK/Trac-ik-cpp/third_party/orocos_kdl/eigen3")
set(EIGEN_INCLUDE_DIR "${EIGEN_ROOT_DIR}")

# ---------- 源码 ----------
set(LIB_SOURCES
    src/kdl_tl.cpp
    src/nlopt_ik.cpp
    src/trac_ik.cpp
)
set(LIB_HEADERS
    include/kdl_tl.h
    include/nlopt_ik.h
    include/trac_ik.h
)

# 1. 库：trac_ik
add_library(trac_ik STATIC ${LIB_SOURCES} ${LIB_HEADERS})
target_include_directories(trac_ik PRIVATE
    ${NLOPT_HPP_DIR}
    ${NLOPT_H_DIR}
    ${KDL_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
)
target_link_directories(trac_ik PRIVATE 
    ${NLOPT_LIB_DIR}
    ${KDL_LIB_DIR}
)
target_link_libraries(trac_ik PRIVATE 
    nlopt
    orocos-kdl
)

# 2. 可执行文件：trac_ik_example
add_executable(trac_ik_example examples/example.cpp)

# 3. trac_ik_example 的 include 路径
target_include_directories(trac_ik_example PRIVATE
    ${NLOPT_HPP_DIR}
    ${NLOPT_H_DIR}
    ${KDL_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
)

# 4. trac_ik_example 的库路径
target_link_directories(trac_ik_example PRIVATE
    ${NLOPT_LIB_DIR}
    ${KDL_LIB_DIR}
)

# 5. trac_ik_example 的链接库
target_link_libraries(trac_ik_example PRIVATE
    trac_ik
    nlopt
    orocos-kdl
)

# ---------- 安装 ----------
install(TARGETS trac_ik trac_ik_example
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)

# ---------- 打印 ----------
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "NLopt include: ${NLOPT_INCLUDE_DIR}")
message(STATUS "NLopt library: ${NLOPT_LIB_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}${NLOPT_LIB}${CMAKE_STATIC_LIBRARY_SUFFIX}")
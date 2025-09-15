# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(TRAC_IK)

set(CMAKE_CXX_STANDARD 17)

# 设置编译模式为 Release（默认开启 -O3 和 -DNDEBUG）
set(CMAKE_BUILD_TYPE Release)

# 强制添加优化选项（确保生效，不受默认配置影响）
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

# 确保 NDEBUG 宏被定义（禁用 assert 等调试代码）
add_definitions(-DNDEBUG)

# ============ 查找依赖 ============
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)
find_package(orocos_kdl REQUIRED)       # Orocos KDL
find_package(urdfdom REQUIRED)          # urdfdom
# find_package(urdfdom_headers REQUIRED)  # urdfdom_headers
find_package(yaml-cpp REQUIRED)         # yaml-cpp
find_package(Eigen3 REQUIRED NO_MODULE) # Eigen3

# 打印信息（调试方便）
message(STATUS "NLOPT found: ${nlopt_DIR}")
message(STATUS "Orocos KDL found: ${orocos_kdl_DIR}")
message(STATUS "urdfdom found: ${urdfdom_INCLUDE_DIRS}")
# message(STATUS "urdfdom_headers found: ${urdfdom_headers_INCLUDE_DIRS}")
message(STATUS "yaml-cpp found: ${yaml-cpp_DIR}")
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

# ============ 头文件目录 ============
include_directories(
    include
    src
    ${EIGEN3_INCLUDE_DIR}
    ${NLOPT_INCLUDE_DIRS}
)

# ============ 源文件 ============
set(LIB_SOURCES
    src/kdl_tl.cpp
    src/nlopt_ik.cpp
    src/trac_ik.cpp
    src/urdf_kdl_converter.cpp
)
set(LIB_HEADERS
    include/kdl_tl.h
    include/nlopt_ik.h
    include/trac_ik.h
    include/urdf_kdl_converter.h
)

# ============ 创建库 ============
add_library(trac_ik ${LIB_SOURCES} ${LIB_HEADERS})

# 链接依赖
target_link_libraries(trac_ik
    PUBLIC
        ${NLOPT_LIBRARIES}
        orocos-kdl
        urdfdom_model
        # urdfdom_headers
        yaml-cpp
        Eigen3::Eigen
)

# ============ 可执行文件 ============
add_executable(trac_ik_example examples/example.cpp)
target_link_libraries(trac_ik_example trac_ik)

add_executable(urdf_to_kdl_example examples/urdf_to_kdl_example.cpp)
target_link_libraries(urdf_to_kdl_example trac_ik)

add_executable(urdf examples/urdf.cpp)
target_link_libraries(urdf trac_ik)

# ============ 设置 RPATH ============
set_target_properties(trac_ik trac_ik_example urdf_to_kdl_example urdf PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH /usr/local/lib
    BUILD_RPATH /usr/local/lib
)

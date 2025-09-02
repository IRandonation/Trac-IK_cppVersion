cmake_minimum_required(VERSION 3.10)
project(TRAC_IK VERSION 1.0.0 LANGUAGES CXX)

set(BUILD_SHARED_LIBS OFF)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
    # 自动检测vcpkg工具链文件
    if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
    endif()

    # 处理Windows特有的库命名规则
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set(CMAKE_DEBUG_POSTFIX "_d")
endif()

# ---------- 源码文件 ----------
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

# 1. 创建静态库 trac_ik
add_library(trac_ik STATIC ${LIB_SOURCES} ${LIB_HEADERS})

# 2. 包含 trac_ik 自己的头文件目录
target_include_directories(trac_ik
    PUBLIC
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        src
)

# 3. 查找 vcpkg 安装的第三方库
find_package(nlopt REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)

# 对于 urdfdom，尝试多种查找方式
find_package(urdfdom QUIET)
if(NOT urdfdom_FOUND)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(URDFDOM urdfdom)
    endif()
endif()

find_package(yaml-cpp CONFIG REQUIRED)

# 打印调试信息
message(STATUS "urdfdom_FOUND: ${urdfdom_FOUND}")
message(STATUS "urdfdom_LIBRARIES: ${urdfdom_LIBRARIES}")
message(STATUS "urdfdom_INCLUDE_DIRS: ${urdfdom_INCLUDE_DIRS}")
message(STATUS "URDFDOM_LIBRARIES: ${URDFDOM_LIBRARIES}")
message(STATUS "URDFDOM_INCLUDE_DIRS: ${URDFDOM_INCLUDE_DIRS}")

# 4. 设置 urdfdom 库变量
if(urdfdom_FOUND)
    # 使用 find_package 找到的库
    set(URDF_LIBRARIES ${urdfdom_LIBRARIES})
    set(URDF_INCLUDE_DIRS ${urdfdom_INCLUDE_DIRS})
elseif(URDFDOM_FOUND)
    # 使用 pkg-config 找到的库
    set(URDF_LIBRARIES ${URDFDOM_LIBRARIES})
    set(URDF_INCLUDE_DIRS ${URDFDOM_INCLUDE_DIRS})
else()
    # 手动查找库文件
    if(WIN32)
        find_library(URDFDOM_MODEL_LIBRARY 
            NAMES urdfdom_model urdfdom_model_d
            PATHS 
                ${CMAKE_PREFIX_PATH}/lib
                ${CMAKE_PREFIX_PATH}/debug/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/debug/lib
        )
        find_library(URDFDOM_SENSOR_LIBRARY 
            NAMES urdfdom_sensor urdfdom_sensor_d
            PATHS 
                ${CMAKE_PREFIX_PATH}/lib
                ${CMAKE_PREFIX_PATH}/debug/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/debug/lib
        )
        find_library(URDFDOM_MODEL_STATE_LIBRARY 
            NAMES urdfdom_model_state urdfdom_model_state_d
            PATHS 
                ${CMAKE_PREFIX_PATH}/lib
                ${CMAKE_PREFIX_PATH}/debug/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/lib
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/debug/lib
        )
        
        set(URDF_LIBRARIES ${URDFDOM_MODEL_LIBRARY} ${URDFDOM_SENSOR_LIBRARY} ${URDFDOM_MODEL_STATE_LIBRARY})
        
        # 查找头文件目录
        find_path(URDF_INCLUDE_DIRS urdf_model/model.h
            PATHS 
                ${CMAKE_PREFIX_PATH}/include
                $ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/include
        )
    else()
        find_library(URDFDOM_MODEL_LIBRARY urdfdom_model)
        find_library(URDFDOM_SENSOR_LIBRARY urdfdom_sensor)
        find_library(URDFDOM_MODEL_STATE_LIBRARY urdfdom_model_state)
        set(URDF_LIBRARIES ${URDFDOM_MODEL_LIBRARY} ${URDFDOM_SENSOR_LIBRARY} ${URDFDOM_MODEL_STATE_LIBRARY})
        
        find_path(URDF_INCLUDE_DIRS urdf_model/model.h)
    endif()
endif()

# 检查是否找到了必要的库
if(NOT URDF_LIBRARIES)
    message(FATAL_ERROR "无法找到 urdfdom 库文件。请确保已正确安装 urdfdom。")
endif()

message(STATUS "最终使用的 URDF 库: ${URDF_LIBRARIES}")
message(STATUS "最终使用的 URDF 头文件目录: ${URDF_INCLUDE_DIRS}")

# 5. 链接库到 trac_ik
target_link_libraries(trac_ik
    PRIVATE
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        urdfdom_headers::urdfdom_headers
        ${URDF_LIBRARIES}
        yaml-cpp::yaml-cpp
)

# 添加包含目录
if(URDF_INCLUDE_DIRS)
    target_include_directories(trac_ik PRIVATE ${URDF_INCLUDE_DIRS})
endif()

# 6. 可执行文件 trac_ik_example
add_executable(trac_ik_example examples/example.cpp)
target_link_libraries(trac_ik_example
    PRIVATE
        trac_ik
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        urdfdom_headers::urdfdom_headers
        ${URDF_LIBRARIES}
        yaml-cpp::yaml-cpp
)

if(URDF_INCLUDE_DIRS)
    target_include_directories(trac_ik_example PRIVATE ${URDF_INCLUDE_DIRS})
endif()

# 7. urdf_to_kdl_example 可执行文件
add_executable(urdf_to_kdl_example examples/urdf_to_kdl_example.cpp)
target_link_libraries(urdf_to_kdl_example
    PRIVATE
        trac_ik
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        urdfdom_headers::urdfdom_headers
        ${URDF_LIBRARIES}
        yaml-cpp::yaml-cpp
)

if(URDF_INCLUDE_DIRS)
    target_include_directories(urdf_to_kdl_example PRIVATE ${URDF_INCLUDE_DIRS})
endif()

# 8. urdf 可执行文件
add_executable(urdf examples/urdf.cpp)
target_link_libraries(urdf
    PRIVATE
        trac_ik
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        urdfdom_headers::urdfdom_headers
        ${URDF_LIBRARIES}
        yaml-cpp::yaml-cpp
)

if(URDF_INCLUDE_DIRS)
    target_include_directories(urdf PRIVATE ${URDF_INCLUDE_DIRS})
endif()

# ---------- 安装规则 ----------
include(GNUInstallDirs)
install(TARGETS trac_ik trac_ik_example urdf_to_kdl_example urdf
    EXPORT TRAC_IKTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
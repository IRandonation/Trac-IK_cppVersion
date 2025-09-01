cmake_minimum_required(VERSION 3.10)
project(TRAC_IK VERSION 1.0.0 LANGUAGES CXX)

set(BUILD_SHARED_LIBS OFF)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 可选：设置编译器（如果用 MinGW，确保 vcpkg 也支持）
if(MINGW)
    set(CMAKE_CXX_COMPILER g++)
    set(CMAKE_C_COMPILER gcc)
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
        include  # 假设你的公共头文件在这里
    PRIVATE
        src      # 如果有一些内部头文件
)


# 3. 查找 vcpkg 安装的第三方库
find_package(nlopt REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdfdom REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# 如果你使用了 Boost 的某些组件，比如 system, filesystem，也可以加上：
# find_package(Boost REQUIRED COMPONENTS system filesystem)

# 4. 链接库到 trac_ik
target_link_libraries(trac_ik
    PRIVATE
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        # Boost::system Boost::filesystem   # 如果用到 Boost
)

# 5. 可执行文件 trac_ik_example
add_executable(trac_ik_example examples/example.cpp)

# 6. 链接 trac_ik 以及第三方库到可执行文件
target_link_libraries(trac_ik_example
    PRIVATE
        trac_ik
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        # Boost::system Boost::filesystem
)

# 7. 添加 urdf_to_kdl_example 可执行文件
add_executable(urdf_to_kdl_example examples/urdf_to_kdl_example.cpp)
target_link_libraries(urdf_to_kdl_example
    PRIVATE
        trac_ik
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
        urdfdom::urdfdom_model
        urdfdom_headers::urdfdom_headers
        yaml-cpp::yaml-cpp
)

# 8. 添加 urdf 可执行文件
add_executable(urdf examples/urdf.cpp)
target_include_directories(urdf
    PRIVATE
        ${urdfdom_headers_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS}
)
target_link_libraries(urdf
    PRIVATE
        urdfdom::urdfdom_model
        yaml-cpp::yaml-cpp
)



# ---------- 安装规则（可选）----------
install(TARGETS trac_ik trac_ik_example urdf_to_kdl_example urdf
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
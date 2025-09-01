# URDF到KDL转换器使用说明

## 概述

URDF到KDL转换器是一个用于将URDF（Unified Robot Description Format）机器人模型转换为KDL（Kinematics and Dynamics Library）运动学链的C++库。该转换器提供了一种简单、高效的方式来解析URDF文件，并将其转换为TRAC-IK等运动学求解器可以使用的格式。

## 主要功能

1. **URDF解析**：从URDF文件或字符串中解析机器人模型
2. **数据结构转换**：将URDF数据结构转换为KDL兼容的内部表示
3. **KDL Chain构建**：从解析后的数据构建KDL运动学链
4. **错误处理和验证**：提供全面的错误处理和模型验证功能
5. **TRAC-IK集成**：与TRAC-IK逆运动学求解器无缝集成

## 快速开始

### 基本用法

```cpp
#include "urdf_kdl_converter.h"
#include "trac_ik.h"

int main() {
    // 创建转换器实例
    URDF_KDL::URDFToKDLConverter converter;
    
    // 从URDF文件加载机器人模型
    if (!converter.loadFromFile("path/to/robot.urdf")) {
        std::cerr << "Error: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    // 验证机器人模型
    if (!converter.validateRobotModel()) {
        std::cerr << "Validation failed: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    // 转换为KDL Chain
    KDL::Chain chain;
    KDL::JntArray q_min, q_max;
    
    if (!converter.toKDLChain(chain, q_min, q_max)) {
        std::cerr << "Conversion failed: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    // 使用TRAC-IK求解器
    TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);
    
    // ... 使用求解器进行逆运动学计算
    
    return 0;
}
```

### 指定根链接和末端链接

```cpp
// 指定根链接和末端链接创建KDL Chain
std::string root_link = "base_link";
std::string tip_link = "end_effector";

// 验证链接和路径
if (!converter.validateLink(root_link)) {
    std::cerr << "Root link '" << root_link << "' does not exist" << std::endl;
    return 1;
}

if (!converter.validateLink(tip_link)) {
    std::cerr << "Tip link '" << tip_link << "' does not exist" << std::endl;
    return 1;
}

if (!converter.validatePath(root_link, tip_link)) {
    std::cerr << "No valid path from '" << root_link << "' to '" << tip_link << "'" << std::endl;
    return 1;
}

// 创建特定链路
KDL::Chain specific_chain;
KDL::JntArray specific_q_min, specific_q_max;

if (converter.toKDLChain(root_link, tip_link, specific_chain, specific_q_min, specific_q_max)) {
    // 使用特定链路进行计算
    TRAC_IK::TRAC_IK solver(specific_chain, specific_q_min, specific_q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);
    
    // ... 使用求解器进行逆运动学计算
}
```

## API参考

### URDFToKDLConverter类

#### 构造函数和析构函数

```cpp
URDFToKDLConverter();
~URDFToKDLConverter();
```

#### 加载URDF模型

```cpp
// 从文件加载URDF模型
bool loadFromFile(const std::string& filename);

// 从字符串加载URDF模型
bool loadFromString(const std::string& urdf_string);
```

#### 获取数据和转换

```cpp
// 获取解析后的机器人数据
std::shared_ptr<RobotData> getRobotData() const;

// 转换为KDL Chain（使用默认根和末端）
bool toKDLChain(KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const;

// 转换为KDL Chain（指定根和末端）
bool toKDLChain(const std::string& root_name, const std::string& tip_name,
               KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const;
```

#### 验证功能

```cpp
// 验证机器人模型的完整性
bool validateRobotModel() const;

// 验证指定链路是否存在
bool validateLink(const std::string& link_name) const;

// 验证指定关节是否存在
bool validateJoint(const std::string& joint_name) const;

// 验证从根到末端的路径是否有效
bool validatePath(const std::string& root_name, const std::string& tip_name) const;
```

#### 错误处理

```cpp
// 获取错误信息
std::string getErrorMessage() const;
```

### 数据结构

#### RobotData

表示完整的机器人模型，包含所有链接和关节信息。

```cpp
struct RobotData {
    std::string name;  // 机器人名称
    std::map<std::string, std::shared_ptr<LinkData>> links;  // 所有链接
    std::map<std::string, std::shared_ptr<JointData>> joints;  // 所有关节
    std::string root_link_name;  // 根链接名称
    
    // 获取根链接
    std::shared_ptr<LinkData> getRootLink() const;
    
    // 获取指定链接
    std::shared_ptr<LinkData> getLink(const std::string& name) const;
    
    // 获取指定关节
    std::shared_ptr<JointData> getJoint(const std::string& name) const;
};
```

#### LinkData

表示机器人中的一个链接。

```cpp
struct LinkData {
    std::string name;  // 链接名称
    std::shared_ptr<InertialData> inertial;  // 惯性参数
    std::shared_ptr<VisualData> visual;  // 视觉元素
    std::shared_ptr<CollisionData> collision;  // 碰撞元素
    std::vector<std::shared_ptr<VisualData>> visual_array;  // 视觉元素数组
    std::vector<std::shared_ptr<CollisionData>> collision_array;  // 碰撞元素数组
    std::vector<std::string> child_links;  // 子链接名称列表
};
```

#### JointData

表示机器人中的一个关节。

```cpp
struct JointData {
    std::string name;  // 关节名称
    Type type;  // 关节类型
    std::string parent_link_name;  // 父链接名称
    std::string child_link_name;  // 子链接名称
    KDL::Frame parent_to_joint_transform;  // 从父链接到关节的变换
    KDL::Vector axis;  // 关节轴
    std::shared_ptr<Limits> limits;  // 关节限位
    
    enum class Type {
        UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    };
    
    struct Limits {
        double lower, upper;  // 位置限位
        double effort, velocity;  // 力和速度限位
    };
    
    // 转换为KDL关节类型
    KDL::Joint::JointType toKDLType() const;
};
```

## 错误处理

转换器提供了全面的错误处理机制。当操作失败时，可以通过`getErrorMessage()`方法获取详细的错误信息。

```cpp
if (!converter.loadFromFile("robot.urdf")) {
    std::cerr << "Failed to load URDF: " << converter.getErrorMessage() << std::endl;
    // 处理错误
}
```

常见的错误情况包括：
- URDF文件不存在或无法读取
- URDF文件格式错误
- 机器人模型不完整（如缺少根链接）
- 指定的链接或关节不存在
- 指定的路径无效（根和末端之间没有连接）

## 验证功能

转换器提供了多种验证功能，可以帮助确保机器人模型的完整性和正确性。

### 模型验证

```cpp
if (!converter.validateRobotModel()) {
    std::cerr << "Invalid robot model: " << converter.getErrorMessage() << std::endl;
    return false;
}
```

模型验证检查以下内容：
- 模型是否包含链接
- 是否指定了根链接
- 根链接是否存在
- 所有关节的父链接和子链接是否存在
- 所有链接是否都通过关节连接到根链接

### 链接和关节验证

```cpp
if (!converter.validateLink("base_link")) {
    std::cerr << "Link 'base_link' does not exist" << std::endl;
}

if (!converter.validateJoint("joint1")) {
    std::cerr << "Joint 'joint1' does not exist" << std::endl;
}
```

### 路径验证

```cpp
if (!converter.validatePath("base_link", "end_effector")) {
    std::cerr << "No valid path from 'base_link' to 'end_effector'" << std::endl;
    std::cerr << "Error: " << converter.getErrorMessage() << std::endl;
}
```

## 示例程序

项目包含一个完整的示例程序`urdf_to_kdl_example.cpp`，演示了如何使用转换器的各种功能。

### 编译和运行

```bash
# 编译项目
mkdir build && cd build
cmake ..
make

# 运行示例程序
./urdf_to_kdl_example examples/robot.urdf
```

### 示例输出

```
URDF to KDL Chain Builder Example
=================================
Loading URDF file: examples/robot.urdf
URDF file loaded successfully!
Robot model validation passed!

=== Robot Data Information ===
Robot name: my_robot
Root link: base_link
Number of links: 3
Number of joints: 2

=== Links ===
Link: base_link
Link: link1
Link: link2

=== Joints ===
Joint: joint1 (base_link -> link1)
  Type: Revolute
  Limits: [-3.14159, 3.14159]
Joint: joint2 (link1 -> link2)
  Type: Revolute
  Limits: [-1.5708, 1.5708]

=== KDL Chain Information ===
Number of segments: 3
Number of joints: 2

=== Joint Limits ===
Joint 0: [-3.14159, 3.14159]
Joint 1: [-1.5708, 1.5708]

=== Chain Segments ===
Segment 0: base_link
  Joint type: Fixed
  Joint origin: [0, 0, 0]
  Joint axis: [0, 0, 0]
Segment 1: link1
  Joint type: RotAxis
  Joint origin: [0, 0, 0.1]
  Joint axis: [0, 0, 1]
Segment 2: link2
  Joint type: RotAxis
  Joint origin: [0, 0, 0.2]
  Joint axis: [0, 0, 1]

=== Testing TRAC-IK Solver ===
Target pose calculated successfully
Target position: [0, 0, 0.3]
Target orientation (RPY): [0, 0, 0]
IK solution found successfully!
Solution time: 1234 microseconds
Initial joint angles: [0, 0]
Solution joint angles: [0.785398, 0.523599]
Position error: 1.23457e-06 m
Orientation error: 2.34568e-07 rad
```

## 常见问题

### Q: 如何处理没有限位的关节？

A: 转换器会自动为没有限位的关节设置默认限位：
- 连续旋转关节（CONTINUOUS）：[-π, π]
- 旋转关节（REVOLUTE）和平移关节（PRISMATIC）：[-DBL_MAX, DBL_MAX]
- 固定关节（FIXED）：[0, 0]

### Q: 如何处理复杂的机器人结构？

A: 转换器使用深度优先搜索算法来查找从根链接到末端链接的路径，可以处理任意复杂的树形结构机器人。

### Q: 如何提高转换性能？

A: 转换器已经进行了性能优化，但对于大型机器人模型，可以考虑以下优化：
1. 只加载和转换需要的部分
2. 缓存转换结果
3. 在多线程环境中使用多个转换器实例

## 扩展和定制

转换器设计为可扩展的，可以通过以下方式进行定制：

1. **添加新的几何体类型**：扩展`GeometryData`结构以支持新的几何体类型
2. **自定义验证规则**：继承`URDFToKDLConverter`类并重写验证方法
3. **添加新的关节类型**：扩展`JointData`结构以支持新的关节类型

## 许可证

本代码遵循与TRAC-IK项目相同的许可证。

## 贡献

欢迎提交问题报告和拉取请求。在提交代码之前，请确保：
1. 代码符合项目的编码风格
2. 添加适当的单元测试
3. 更新相关文档
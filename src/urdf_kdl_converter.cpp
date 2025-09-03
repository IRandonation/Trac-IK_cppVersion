#include "urdf_kdl_converter.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <queue>
#include <limits>
#include <cmath>

namespace URDF_KDL {

// 几何体数据转换
std::shared_ptr<GeometryData> GeometryData::fromURDF(const urdf::GeometrySharedPtr& geom) {
    if (!geom) return nullptr;
    
    auto geom_data = std::make_shared<GeometryData>();
    
    switch (geom->type) {
        case urdf::Geometry::SPHERE: {
            geom_data->type = Type::SPHERE;
            auto sphere = std::static_pointer_cast<urdf::Sphere>(geom);
            geom_data->sphere.radius = sphere->radius;
            break;
        }
        case urdf::Geometry::BOX: {
            geom_data->type = Type::BOX;
            auto box = std::static_pointer_cast<urdf::Box>(geom);
            geom_data->box.x = box->dim.x;
            geom_data->box.y = box->dim.y;
            geom_data->box.z = box->dim.z;
            break;
        }
        case urdf::Geometry::CYLINDER: {
            geom_data->type = Type::CYLINDER;
            auto cylinder = std::static_pointer_cast<urdf::Cylinder>(geom);
            geom_data->cylinder.radius = cylinder->radius;
            geom_data->cylinder.length = cylinder->length;
            break;
        }
        case urdf::Geometry::MESH: {
            geom_data->type = Type::MESH;
            auto mesh = std::static_pointer_cast<urdf::Mesh>(geom);
            geom_data->mesh.filename = mesh->filename;
            geom_data->mesh.scale_x = mesh->scale.x;
            geom_data->mesh.scale_y = mesh->scale.y;
            geom_data->mesh.scale_z = mesh->scale.z;
            break;
        }
        default:
            geom_data->type = Type::UNKNOWN;
            break;
    }
    
    return geom_data;
}

// 视觉数据转换
std::shared_ptr<VisualData> VisualData::fromURDF(const urdf::VisualSharedPtr& visual) {
    if (!visual) return nullptr;
    
    auto visual_data = std::make_shared<VisualData>();
    
    // 转换位姿
    visual_data->origin.p.x(visual->origin.position.x);
    visual_data->origin.p.y(visual->origin.position.y);
    visual_data->origin.p.z(visual->origin.position.z);
    visual_data->origin.M = KDL::Rotation::Quaternion(
        visual->origin.rotation.x,
        visual->origin.rotation.y,
        visual->origin.rotation.z,
        visual->origin.rotation.w
    );
    
    // 转换几何体
    visual_data->geometry = GeometryData::fromURDF(visual->geometry);
    
    // 转换材质
    if (visual->material) {
        visual_data->material = std::make_shared<Material>();
        visual_data->material->name = visual->material->name;
        visual_data->material->color.r = visual->material->color.r;
        visual_data->material->color.g = visual->material->color.g;
        visual_data->material->color.b = visual->material->color.b;
        visual_data->material->color.a = visual->material->color.a;
        visual_data->material->texture_filename = visual->material->texture_filename;
    }
    
    visual_data->material_name = visual->material_name;
    
    return visual_data;
}

// 碰撞数据转换
std::shared_ptr<CollisionData> CollisionData::fromURDF(const urdf::CollisionSharedPtr& collision) {
    if (!collision) return nullptr;
    
    auto collision_data = std::make_shared<CollisionData>();
    
    // 转换位姿
    collision_data->origin.p.x(collision->origin.position.x);
    collision_data->origin.p.y(collision->origin.position.y);
    collision_data->origin.p.z(collision->origin.position.z);
    collision_data->origin.M = KDL::Rotation::Quaternion(
        collision->origin.rotation.x,
        collision->origin.rotation.y,
        collision->origin.rotation.z,
        collision->origin.rotation.w
    );
    
    // 转换几何体
    collision_data->geometry = GeometryData::fromURDF(collision->geometry);
    
    return collision_data;
}

// 惯性数据转换
std::shared_ptr<InertialData> InertialData::fromURDF(const urdf::InertialSharedPtr& inertial) {
    if (!inertial) return nullptr;
    
    auto inertial_data = std::make_shared<InertialData>();
    
    // 转换位姿
    inertial_data->origin.p.x(inertial->origin.position.x);
    inertial_data->origin.p.y(inertial->origin.position.y);
    inertial_data->origin.p.z(inertial->origin.position.z);
    inertial_data->origin.M = KDL::Rotation::Quaternion(
        inertial->origin.rotation.x,
        inertial->origin.rotation.y,
        inertial->origin.rotation.z,
        inertial->origin.rotation.w
    );
    
    // 转换质量和惯性张量
    inertial_data->mass = inertial->mass;
    inertial_data->ixx = inertial->ixx;
    inertial_data->ixy = inertial->ixy;
    inertial_data->ixz = inertial->ixz;
    inertial_data->iyy = inertial->iyy;
    inertial_data->iyz = inertial->iyz;
    inertial_data->izz = inertial->izz;
    
    return inertial_data;
}

// 关节数据转换
std::shared_ptr<JointData> JointData::fromURDF(const urdf::JointSharedPtr& joint) {
    if (!joint) return nullptr;
    
    auto joint_data = std::make_shared<JointData>();
    
    // 基本属性
    joint_data->name = joint->name;
    joint_data->type = static_cast<JointData::Type>(joint->type);
    joint_data->parent_link_name = joint->parent_link_name;
    joint_data->child_link_name = joint->child_link_name;
    
    // 转换位姿
    joint_data->parent_to_joint_transform.p.x(joint->parent_to_joint_origin_transform.position.x);
    joint_data->parent_to_joint_transform.p.y(joint->parent_to_joint_origin_transform.position.y);
    joint_data->parent_to_joint_transform.p.z(joint->parent_to_joint_origin_transform.position.z);
    joint_data->parent_to_joint_transform.M = KDL::Rotation::Quaternion(
        joint->parent_to_joint_origin_transform.rotation.x,
        joint->parent_to_joint_origin_transform.rotation.y,
        joint->parent_to_joint_origin_transform.rotation.z,
        joint->parent_to_joint_origin_transform.rotation.w
    );
    
    // 转换关节轴
    joint_data->axis.x(joint->axis.x);
    joint_data->axis.y(joint->axis.y);
    joint_data->axis.z(joint->axis.z);
    
    // 转换关节限位
    if (joint->limits) {
        joint_data->limits = std::make_shared<Limits>();
        joint_data->limits->lower = joint->limits->lower;
        joint_data->limits->upper = joint->limits->upper;
        joint_data->limits->effort = joint->limits->effort;
        joint_data->limits->velocity = joint->limits->velocity;
    }
    
    return joint_data;
}

// 转换为KDL关节类型
KDL::Joint::JointType JointData::toKDLType() const {
    switch (type) {
        case Type::REVOLUTE:
            return KDL::Joint::RotAxis;
        case Type::CONTINUOUS:
            return KDL::Joint::RotAxis;
        case Type::PRISMATIC:
            return KDL::Joint::TransAxis;
        case Type::FLOATING:
            return KDL::Joint::None;
        case Type::PLANAR:
            return KDL::Joint::None;
        case Type::FIXED:
            return KDL::Joint::None;
        default:
            return KDL::Joint::None;
    }
}

// 链接数据转换
std::shared_ptr<LinkData> LinkData::fromURDF(const urdf::LinkSharedPtr& link) {
    if (!link) {
        std::cerr << "[LinkData::fromURDF] Warning: Received null link pointer." << std::endl;
        return nullptr;
    }

    auto link_data = std::make_shared<LinkData>();

    // --- 基本属性 ---
    link_data->name = link->name;
    std::cout << "[LinkData::fromURDF] Converting link: " << link_data->name << std::endl;

    // --- 惯性参数 ---
    if (link->inertial) {
        link_data->inertial = InertialData::fromURDF(link->inertial);
    } else {
        link_data->inertial = nullptr;
    }

    // --- 视觉元素（单个）---
    if (link->visual) {
        link_data->visual = VisualData::fromURDF(link->visual);
    } else {
        link_data->visual = nullptr;
    }

    // --- 碰撞元素（单个）---
    if (link->collision) {
        link_data->collision = CollisionData::fromURDF(link->collision);
    } else {
        link_data->collision = nullptr;
    }

    // --- 视觉元素数组（多个）---
    for (const auto& visual : link->visual_array) {
        if (visual) {
            auto visual_data = VisualData::fromURDF(visual);
            link_data->visual_array.push_back(visual_data);
        } else {
            std::cerr << "[LinkData::fromURDF] Warning: Null visual element in visual_array for link: " 
                      << link_data->name << std::endl;
            link_data->visual_array.push_back(nullptr);  // 保持数组结构一致，或者可以选择不插入
        }
    }

    // --- 碰撞元素数组（多个）---
    for (const auto& collision : link->collision_array) {
        if (collision) {
            auto collision_data = CollisionData::fromURDF(collision);
            link_data->collision_array.push_back(collision_data);
        } else {
            std::cerr << "[LinkData::fromURDF] Warning: Null collision element in collision_array for link: " 
                      << link_data->name << std::endl;
            link_data->collision_array.push_back(nullptr);  // 保持数组结构一致
        }
    }

    // --- 子链接名称列表 ---
    for (const auto& child_link : link->child_links) {
        if (child_link) {
            link_data->child_links.push_back(child_link->name);
        } else {
            std::cerr << "[LinkData::fromURDF] Warning: Null child link detected for link: " 
                      << link_data->name << std::endl;
        }
    }

    return link_data;
}

// 机器人数据转换
std::shared_ptr<RobotData> RobotData::fromURDF(const urdf::ModelInterfaceSharedPtr& model) {
    if (!model) {
        std::cout << "NO URDF" << std::endl;

        return nullptr;
    }
    std::cout << "Starting URDF to RobotData conversion..." << std::endl;

    auto robot_data = std::make_shared<RobotData>();
    
    // 基本属性
    robot_data->name = model->getName();

    std::cout << "Converting links..." << std::endl;
    
    // 转换链接
    for (const auto& link_pair : model->links_) {
        std::cout << "Converting link: " << link_pair.first << std::endl;
        robot_data->links[link_pair.first] = LinkData::fromURDF(link_pair.second);
    }

    std::cout << "Converting joints..." << std::endl;
    
    // 转换关节
    for (const auto& joint_pair : model->joints_) {
        robot_data->joints[joint_pair.first] = JointData::fromURDF(joint_pair.second);
    }
    
    // 设置根链接
    if (model->getRoot()) {
        robot_data->root_link_name = model->getRoot()->name;
    }

    std::cout << "Conversion successful." << std::endl;

    
    return robot_data;
}

// 获取根链接
std::shared_ptr<LinkData> RobotData::getRootLink() const {
    auto it = links.find(root_link_name);
    if (it != links.end()) {
        return it->second;
    }
    return nullptr;
}

// 获取指定链接
std::shared_ptr<LinkData> RobotData::getLink(const std::string& name) const {
    auto it = links.find(name);
    if (it != links.end()) {
        return it->second;
    }
    return nullptr;
}

// 获取指定关节
std::shared_ptr<JointData> RobotData::getJoint(const std::string& name) const {
    auto it = joints.find(name);
    if (it != joints.end()) {
        return it->second;
    }
    return nullptr;
}

// URDF到KDL转换器实现
URDFToKDLConverter::URDFToKDLConverter() : robot_data_(nullptr) {
}

URDFToKDLConverter::~URDFToKDLConverter() {
}

// 从URDF文件加载机器人模型
bool URDFToKDLConverter::loadFromFile(const std::string& filename) {
    try {
        auto model = urdf::parseURDFFile(filename);
        if (!model) {
            setErrorMessage("Failed to parse URDF file: " + filename);
            return false;
        }
        
        robot_data_ = RobotData::fromURDF(model);
        if (!robot_data_) {
            setErrorMessage("Failed to convert URDF model to internal representation");
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        setErrorMessage(std::string("Exception while loading URDF file: ") + e.what());
        return false;
    }
}

// 从URDF字符串加载机器人模型
bool URDFToKDLConverter::loadFromString(const std::string& urdf_string) {
    try {
        auto model = urdf::parseURDF(urdf_string);
        if (!model) {
            setErrorMessage("Failed to parse URDF string");
            return false;
        }
        
        robot_data_ = RobotData::fromURDF(model);
        if (!robot_data_) {
            setErrorMessage("Failed to convert URDF model to internal representation");
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        setErrorMessage(std::string("Exception while parsing URDF string: ") + e.what());
        return false;
    }
}

// 获取解析后的机器人数据
std::shared_ptr<RobotData> URDFToKDLConverter::getRobotData() const {
    return robot_data_;
}

// 转换为KDL Chain
bool URDFToKDLConverter::toKDLChain(KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    // 验证机器人模型
    if (!validateRobotModel()) {
        return false;
    }
    
    // 使用根链接作为默认根
    std::string root_name = robot_data_->root_link_name;
    if (root_name.empty()) {
        setErrorMessage("No root link specified");
        return false;
    }
    
    // 查找末端链接（没有子链接的链接）
    std::string tip_name;
    for (const auto& link_pair : robot_data_->links) {
        if (link_pair.second->child_links.empty()) {
            tip_name = link_pair.first;
            break;
        }
    }
    
    if (tip_name.empty()) {
        setErrorMessage("No tip link found");
        return false;
    }
    
    return toKDLChain(root_name, tip_name, chain, q_min, q_max);
}

// 转换为KDL Chain（指定根链接和末端链接）
bool URDFToKDLConverter::toKDLChain(const std::string& root_name, const std::string& tip_name,
                                   KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    // 验证机器人模型
    if (!validateRobotModel()) {
        return false;
    }
    
    // 验证路径
    if (!validatePath(root_name, tip_name)) {
        return false;
    }
    
    // 查找从根到末端的路径
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
    
    if (!findPath(root_name, tip_name, link_names, joint_names)) {
        setErrorMessage("No path found from '" + root_name + "' to '" + tip_name + "'");
        return false;
    }
    
    // 构建KDL Chain
    chain = KDL::Chain();
    
    // 添加第一个链接（固定链接）
    chain.addSegment(KDL::Segment(root_name, KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));
    
    // 添加关节和链接
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const auto& joint_name = joint_names[i];
        const auto& joint_data = robot_data_->joints.at(joint_name);
        const auto& child_link_data = robot_data_->links.at(joint_data->child_link_name);
    
    KDL::Vector joint_origin_p(
        joint_data->parent_to_joint_transform.p.x(),
        joint_data->parent_to_joint_transform.p.y(),
        joint_data->parent_to_joint_transform.p.z()
    );

    // 使用解析出来的真实旋转
    KDL::Rotation joint_origin_M = joint_data->parent_to_joint_transform.M;

    // 创建真正的关节原点偏移向量
    KDL::Vector joint_offset(joint_origin_p);

    // 创建 KDL::Joint
    KDL::Joint kdl_joint(
        joint_data->name,
        joint_offset,           // ✅ 使用真实偏移，不再是 Zero()
        joint_data->axis,
        joint_data->toKDLType()
    );

    // 创建该 Segment 的真实位姿（关节相对于父链接的变换）
    KDL::Frame joint_frame(joint_origin_M, joint_offset);

    // 创建 KDL::Segment
    KDL::Segment segment(
        child_link_data->name,
        kdl_joint,
        joint_frame  // ✅ 使用真实位姿，不再是 Identity()
    );
        
        chain.addSegment(segment);
    }
    
    // 设置关节限位
    int num_joints = chain.getNrOfJoints();
    q_min.resize(num_joints);
    q_max.resize(num_joints);
    
    for (int i = 0; i < num_joints; ++i) {
        const auto& joint_name = joint_names[i];
        const auto& joint = robot_data_->joints.at(joint_name);
        
        if (joint->limits) {
            q_min(i) = joint->limits->lower;
            q_max(i) = joint->limits->upper;
        } else {
            // 没有限位的关节
            switch (joint->type) {
                case JointData::Type::CONTINUOUS:
                    q_min(i) = -3.14159265358979323846;
                    q_max(i) = 3.14159265358979323846;
                    break;
                case JointData::Type::REVOLUTE:
                case JointData::Type::PRISMATIC:
                    q_min(i) = -std::numeric_limits<double>::max();
                    q_max(i) = std::numeric_limits<double>::max();
                    break;
                default:
                    q_min(i) = 0.0;
                    q_max(i) = 0.0;
                    break;
            }
        }
    }
    
    return true;
}

// 查找从根到末端的路径
bool URDFToKDLConverter::findPath(const std::string& root_name, const std::string& tip_name,
                                std::vector<std::string>& link_names, std::vector<std::string>& joint_names) const {
    link_names.clear();
    joint_names.clear();
    
    // 使用深度优先搜索查找路径
    std::vector<std::string> path_links;
    std::vector<std::string> path_joints;
    std::map<std::string, bool> visited;
    
    // 递归搜索函数
    std::function<bool(const std::string&)> search = [&](const std::string& current) -> bool {
        visited[current] = true;
        path_links.push_back(current);
        
        if (current == tip_name) {
            return true;
        }
        
        // 获取当前链接
        auto current_link = robot_data_->getLink(current);
        if (!current_link) {
            return false;
        }
        
        // 遍历所有关节，找到以当前链接为父链接的关节
        for (const auto& joint_pair : robot_data_->joints) {
            const auto& joint = joint_pair.second;
            if (joint->parent_link_name == current) {
                std::string child_name = joint->child_link_name;
                if (visited.find(child_name) == visited.end() || !visited[child_name]) {
                    path_joints.push_back(joint->name);
                    if (search(child_name)) {
                        return true;
                    }
                    path_joints.pop_back();
                }
            }
        }
        
        path_links.pop_back();
        return false;
    };
    
    // 从根链接开始搜索
    if (search(root_name)) {
        link_names = path_links;
        joint_names = path_joints;
        return true;
    }
    
    return false;
}

// 获取错误信息
std::string URDFToKDLConverter::getErrorMessage() const {
    return error_message_;
}

// 验证机器人模型的完整性
bool URDFToKDLConverter::validateRobotModel() const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    // 检查是否有链接
    if (robot_data_->links.empty()) {
        setErrorMessage("No links found in robot model");
        return false;
    }
    
    // 检查是否有根链接
    if (robot_data_->root_link_name.empty()) {
        setErrorMessage("No root link specified");
        return false;
    }
    
    // 检查根链接是否存在
    if (robot_data_->links.find(robot_data_->root_link_name) == robot_data_->links.end()) {
        setErrorMessage("Root link '" + robot_data_->root_link_name + "' not found");
        return false;
    }
    
    // 检查所有关节的父链接和子链接是否存在
    for (const auto& joint_pair : robot_data_->joints) {
        const auto& joint = joint_pair.second;
        
        if (robot_data_->links.find(joint->parent_link_name) == robot_data_->links.end()) {
            setErrorMessage("Parent link '" + joint->parent_link_name + "' of joint '" + joint->name + "' not found");
            return false;
        }
        
        if (robot_data_->links.find(joint->child_link_name) == robot_data_->links.end()) {
            setErrorMessage("Child link '" + joint->child_link_name + "' of joint '" + joint->name + "' not found");
            return false;
        }
    }
    
    // 检查所有链接是否都通过关节连接
    std::map<std::string, bool> connected;
    connected[robot_data_->root_link_name] = true;
    
    // 使用广度优先搜索检查所有链接是否都连接
    std::queue<std::string> queue;
    queue.push(robot_data_->root_link_name);
    
    while (!queue.empty()) {
        std::string current = queue.front();
        queue.pop();
        
        // 找到所有以当前链接为父链接的关节
        for (const auto& joint_pair : robot_data_->joints) {
            const auto& joint = joint_pair.second;
            if (joint->parent_link_name == current) {
                std::string child = joint->child_link_name;
                if (connected.find(child) == connected.end()) {
                    connected[child] = true;
                    queue.push(child);
                }
            }
        }
    }
    
    // 检查是否有未连接的链接
    for (const auto& link_pair : robot_data_->links) {
        if (connected.find(link_pair.first) == connected.end()) {
            setErrorMessage("Link '" + link_pair.first + "' is not connected to the root link");
            return false;
        }
    }
    
    return true;
}

// 验证指定链路是否存在
bool URDFToKDLConverter::validateLink(const std::string& link_name) const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    return robot_data_->links.find(link_name) != robot_data_->links.end();
}

// 验证指定关节是否存在
bool URDFToKDLConverter::validateJoint(const std::string& joint_name) const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    return robot_data_->joints.find(joint_name) != robot_data_->joints.end();
}

// 验证从根到末端的路径是否有效
bool URDFToKDLConverter::validatePath(const std::string& root_name, const std::string& tip_name) const {
    if (!robot_data_) {
        setErrorMessage("No robot data loaded");
        return false;
    }
    
    // 检查根链接和末端链接是否存在
    if (!validateLink(root_name)) {
        setErrorMessage("Root link '" + root_name + "' not found");
        return false;
    }
    
    if (!validateLink(tip_name)) {
        setErrorMessage("Tip link '" + tip_name + "' not found");
        return false;
    }
    
    // 检查是否存在路径
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
    
    if (!findPath(root_name, tip_name, link_names, joint_names)) {
        setErrorMessage("No path found from '" + root_name + "' to '" + tip_name + "'");
        return false;
    }
    
    return true;
}

// 设置错误信息
void URDFToKDLConverter::setErrorMessage(const std::string& message) const {
    const_cast<URDFToKDLConverter*>(this)->error_message_ = message;
}

}  // namespace URDF_KDL
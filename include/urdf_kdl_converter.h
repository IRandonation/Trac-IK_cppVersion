#ifndef URDF_KDL_CONVERTER_H
#define URDF_KDL_CONVERTER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

namespace URDF_KDL {

/**
 * @brief URDF几何体数据的KDL兼容表示
 */
struct GeometryData {
    enum class Type { SPHERE, BOX, CYLINDER, MESH, UNKNOWN };
    
    Type type;
    
    // 球体参数
    struct Sphere {
        double radius;
    };
    
    // 立方体参数
    struct Box {
        double x, y, z;
    };
    
    // 圆柱体参数
    struct Cylinder {
        double radius, length;
    };
    
    // 网格参数
    struct Mesh {
        std::string filename;
        double scale_x, scale_y, scale_z;
    };
    
    union {
        Sphere sphere;
        Box box;
        Cylinder cylinder;
        Mesh mesh;
    };
    
    GeometryData() : type(Type::UNKNOWN) {}
    ~GeometryData() {}  // 联合体不需要显式析构
    
    // 从URDF几何体创建
    static std::shared_ptr<GeometryData> fromURDF(const urdf::GeometrySharedPtr& geom);
};

/**
 * @brief URDF视觉元素的KDL兼容表示
 */
struct VisualData {
    KDL::Frame origin;  // 视觉元素的位姿
    std::shared_ptr<GeometryData> geometry;
    std::string material_name;
    
    struct Material {
        std::string name;
        struct Color {
            double r, g, b, a;
        } color;
        std::string texture_filename;
    };
    
    std::shared_ptr<Material> material;
    
    // 从URDF视觉元素创建
    static std::shared_ptr<VisualData> fromURDF(const urdf::VisualSharedPtr& visual);
};

/**
 * @brief URDF碰撞元素的KDL兼容表示
 */
struct CollisionData {
    KDL::Frame origin;  // 碰撞元素的位姿
    std::shared_ptr<GeometryData> geometry;
    
    // 从URDF碰撞元素创建
    static std::shared_ptr<CollisionData> fromURDF(const urdf::CollisionSharedPtr& collision);
};

/**
 * @brief URDF惯性参数的KDL兼容表示
 */
struct InertialData {
    KDL::Frame origin;  // 惯性坐标系位姿
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;  // 惯性张量
    
    // 从URDF惯性参数创建
    static std::shared_ptr<InertialData> fromURDF(const urdf::InertialSharedPtr& inertial);
};

/**
 * @brief URDF关节的KDL兼容表示
 */
struct JointData {
    enum class Type {
        UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    };
    
    std::string name;
    Type type;
    std::string parent_link_name;
    std::string child_link_name;
    KDL::Frame parent_to_joint_transform;  // 从父链接到关节的变换
    KDL::Vector axis;  // 关节轴
    
    struct Limits {
        double lower, upper;
        double effort, velocity;
    };
    
    std::shared_ptr<Limits> limits;
    
    // 从URDF关节创建
    static std::shared_ptr<JointData> fromURDF(const urdf::JointSharedPtr& joint);
    
    // 转换为KDL关节类型
    KDL::Joint::JointType toKDLType() const;
};

/**
 * @brief URDF链接的KDL兼容表示
 */
struct LinkData {
    std::string name;
    std::shared_ptr<InertialData> inertial;
    std::shared_ptr<VisualData> visual;
    std::shared_ptr<CollisionData> collision;
    std::vector<std::shared_ptr<VisualData>> visual_array;
    std::vector<std::shared_ptr<CollisionData>> collision_array;
    std::vector<std::string> child_links;  // 子链接名称列表
    
    // 从URDF链接创建
    static std::shared_ptr<LinkData> fromURDF(const urdf::LinkSharedPtr& link);
};

/**
 * @brief URDF机器人的KDL兼容表示
 */
struct RobotData {
    std::string name;
    std::map<std::string, std::shared_ptr<LinkData>> links;
    std::map<std::string, std::shared_ptr<JointData>> joints;
    std::string root_link_name;
    
    // 从URDF模型创建
    static std::shared_ptr<RobotData> fromURDF(const urdf::ModelInterfaceSharedPtr& model);
    
    // 获取根链接
    std::shared_ptr<LinkData> getRootLink() const;
    
    // 获取指定链接
    std::shared_ptr<LinkData> getLink(const std::string& name) const;
    
    // 获取指定关节
    std::shared_ptr<JointData> getJoint(const std::string& name) const;
};

/**
 * @brief URDF到KDL转换器
 */
class URDFToKDLConverter {
public:
    URDFToKDLConverter();
    ~URDFToKDLConverter();
    
    /**
     * @brief 从URDF文件加载机器人模型
     * @param filename URDF文件路径
     * @return 是否成功
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief 从URDF字符串加载机器人模型
     * @param urdf_string URDF字符串
     * @return 是否成功
     */
    bool loadFromString(const std::string& urdf_string);
    
    /**
     * @brief 获取解析后的机器人数据
     * @return 机器人数据
     */
    std::shared_ptr<RobotData> getRobotData() const;
    
    /**
     * @brief 转换为KDL Chain
     * @param chain 输出的KDL Chain
     * @param q_min 输出的关节最小值
     * @param q_max 输出的关节最大值
     * @return 是否成功
     */
    bool toKDLChain(KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const;
    
    /**
     * @brief 转换为KDL Chain（指定根链接和末端链接）
     * @param root_name 根链接名称
     * @param tip_name 末端链接名称
     * @param chain 输出的KDL Chain
     * @param q_min 输出的关节最小值
     * @param q_max 输出的关节最大值
     * @return 是否成功
     */
    bool toKDLChain(const std::string& root_name, const std::string& tip_name,
                   KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max) const;
    
    /**
     * @brief 获取错误信息
     * @return 错误信息
     */
    std::string getErrorMessage() const;
    
    /**
     * @brief 验证机器人模型的完整性
     * @return 是否有效
     */
    bool validateRobotModel() const;
    
    /**
     * @brief 验证指定链路是否存在
     * @param link_name 链路名称
     * @return 是否存在
     */
    bool validateLink(const std::string& link_name) const;
    
    /**
     * @brief 验证指定关节是否存在
     * @param joint_name 关节名称
     * @return 是否存在
     */
    bool validateJoint(const std::string& joint_name) const;
    
    /**
     * @brief 验证从根到末端的路径是否有效
     * @param root_name 根链路名称
     * @param tip_name 末端链路名称
     * @return 是否有效
     */
    bool validatePath(const std::string& root_name, const std::string& tip_name) const;
    
private:
    std::shared_ptr<RobotData> robot_data_;
    mutable std::string error_message_;
    
    // 递归构建KDL Chain
    bool buildKDLChainRecursive(const std::string& root_name, const std::string& tip_name,
                                KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max,
                                std::vector<std::string>& joint_names) const;
    
    // 查找从根到末端的路径
    bool findPath(const std::string& root_name, const std::string& tip_name,
                 std::vector<std::string>& link_names, std::vector<std::string>& joint_names) const;
    
    // 设置错误信息
    void setErrorMessage(const std::string& message) const;
};

}  // namespace URDF_KDL

#endif  // URDF_KDL_CONVERTER_H
#include <iostream>
#include <fstream>
#include <memory>
#include <urdf_parser/urdf_parser.h>   // urdfdom
#include <yaml-cpp/yaml.h>

// 把 urdf::Joint::Type 转字符串
static const char* jointTypeStr(int t)
{
    switch (t)
    {
        case urdf::Joint::REVOLUTE:  return "revolute";
        case urdf::Joint::PRISMATIC: return "prismatic";
        case urdf::Joint::FIXED:     return "fixed";
        case urdf::Joint::CONTINUOUS:return "continuous";
        case urdf::Joint::PLANAR:    return "planar";
        case urdf::Joint::FLOATING:  return "floating";
        default:                     return "unknown";
    }
}

// ---------------- 把 URDF 对象转成 YAML ----------------
static YAML::Node vec2yaml(const urdf::Vector3& v)
{
    YAML::Node n;
    n["x"] = v.x; n["y"] = v.y; n["z"] = v.z;
    return n;
}

static YAML::Node pose2yaml(const urdf::Pose& p)
{
    YAML::Node n;
    n["position"]  = vec2yaml(p.position);
    YAML::Node q;
    q["x"] = p.rotation.x; q["y"] = p.rotation.y;
    q["z"] = p.rotation.z; q["w"] = p.rotation.w;
    n["orientation"] = q;
    return n;
}

static YAML::Node inertia2yaml(const urdf::Inertial& I)
{
    YAML::Node n;
    n["mass"]   = I.mass;
    n["origin"] = pose2yaml(I.origin);
    YAML::Node i;
    i["ixx"] = I.ixx; i["ixy"] = I.ixy; i["ixz"] = I.ixz;
    i["iyy"] = I.iyy; i["iyz"] = I.iyz; i["izz"] = I.izz;
    n["inertia"] = i;
    return n;
}

static YAML::Node geometry2yaml(const urdf::Geometry& g)
{
    YAML::Node n;
    switch (g.type)
    {
        case urdf::Geometry::BOX:
            n["type"] = "box";
            n["size"] = vec2yaml(static_cast<const urdf::Box&>(g).dim);
            break;
        case urdf::Geometry::CYLINDER:
            n["type"] = "cylinder";
            n["radius"] = static_cast<const urdf::Cylinder&>(g).radius;
            n["length"] = static_cast<const urdf::Cylinder&>(g).length;
            break;
        case urdf::Geometry::SPHERE:
            n["type"] = "sphere";
            n["radius"] = static_cast<const urdf::Sphere&>(g).radius;
            break;
        case urdf::Geometry::MESH:
            n["type"] = "mesh";
            n["filename"] = static_cast<const urdf::Mesh&>(g).filename;
            n["scale"] = vec2yaml(static_cast<const urdf::Mesh&>(g).scale);
            break;
        default:
            n["type"] = "unknown";
    }
    return n;
}

static YAML::Node visual2yaml(const urdf::Visual& v) {
    YAML::Node n;
    n["origin"]   = pose2yaml(v.origin);
    n["geometry"] = geometry2yaml(*v.geometry);
    if (v.material) {
        n["material"]["name"] = v.material->name;
        if (v.material) {
            n["material"]["name"] = v.material->name;
            // color 现在是一个值对象，直接访问
            const urdf::Color& c = v.material->color;
            n["material"]["color"]["r"] = c.r;
            n["material"]["color"]["g"] = c.g;
            n["material"]["color"]["b"] = c.b;
            n["material"]["color"]["a"] = c.a;
        }
    }
    return n;
}

static YAML::Node collision2yaml(const urdf::Collision& c)
{
    YAML::Node n;
    n["origin"]   = pose2yaml(c.origin);
    n["geometry"] = geometry2yaml(*c.geometry);
    return n;
}

static YAML::Node link2yaml(const urdf::Link& l)
{
    YAML::Node n;
    n["name"] = l.name;
    if (l.inertial)  n["inertial"]  = inertia2yaml(*l.inertial);
    if (l.visual)    n["visual"]    = visual2yaml(*l.visual);
    if (l.collision) n["collision"] = collision2yaml(*l.collision);
    return n;
}

static YAML::Node joint2yaml(const urdf::Joint& j)
{
    YAML::Node n;
    n["name"]   = j.name;
    n["type"]   = jointTypeStr(j.type);
    n["parent"] = j.parent_link_name;
    n["child"]  = j.child_link_name;
    n["origin"] = pose2yaml(j.parent_to_joint_origin_transform);
    n["axis"]   = vec2yaml(j.axis);
    if (j.limits)
    {
        n["limits"]["lower"]   = j.limits->lower;
        n["limits"]["upper"]   = j.limits->upper;
        n["limits"]["effort"]  = j.limits->effort;
        n["limits"]["velocity"]= j.limits->velocity;
    }
    return n;
}

// ---------------- main ----------------
int main(int argc, char** argv)
{
    const std::string urdf_path = R"(D:\Project\JD_Robot\RobotArm\TRAC-IK\Trac-ik-cpp\examples\robot.urdf)";
    const std::string yaml_path = R"(D:\Project\JD_Robot\RobotArm\TRAC-IK\Trac-ik-cpp\examples\robot_out.yaml)";

    auto model = urdf::parseURDFFile(urdf_path);
    if (!model)
    {
        std::cerr << "Cannot load URDF from " << urdf_path << '\n';
        return 1;
    }

    YAML::Node root;
    for (const auto& kv : model->links_)
        root["links"][kv.first] = link2yaml(*kv.second);

    for (const auto& kv : model->joints_)
        root["joints"][kv.first] = joint2yaml(*kv.second);

    std::ofstream fout(yaml_path);
    fout << root;
    std::cout << "YAML exported to " << yaml_path << '\n';
    return 0;
}
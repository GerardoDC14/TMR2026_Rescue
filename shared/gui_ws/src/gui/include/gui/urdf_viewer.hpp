#pragma once

#include <QOpenGLWidget>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

class UrdfViewer : public QOpenGLWidget, protected QOpenGLExtraFunctions {
    Q_OBJECT
public:
    explicit UrdfViewer(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~UrdfViewer() override;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    // Geometry vertex
    struct Vertex {
        float pos[3];
        float normal[3];
    };

    // Geometry description (parsed from URDF, no GL resources)
    struct GeomDesc {
        enum Type { BOX, CYLINDER, SPHERE, MESH };
        Type type{BOX};
        float dims[3]{0, 0, 0};
        std::string mesh_uri;
        QMatrix4x4 visual_origin;
        QVector4D color{0.7f, 0.7f, 0.7f, 1.0f};
    };

    // GL renderable object
    struct RenderObject {
        GLuint vao{0}, vbo{0};
        int vertex_count{0};
        QVector4D color{0.7f, 0.7f, 0.7f, 1.0f};
        QMatrix4x4 visual_origin;
    };

    struct LinkData {
        std::vector<RenderObject> visuals;
        QMatrix4x4 world_transform;
    };

    struct JointData {
        std::string parent_link;
        std::string child_link;
        int type{6}; // urdf::Joint::FIXED
        QMatrix4x4 origin;
        QVector3D axis{0, 0, 1};
        double value{0.0};
    };

    // URDF parsing (runs on ROS thread, stores descriptions)
    void parseUrdf(const std::string& urdf_xml);

    // GL resource creation (runs on main/GL thread)
    void buildGLResources();
    void cleanupGLResources();

    // Forward kinematics
    void computeFKRecursive(const std::string& link_name,
                            const QMatrix4x4& parent_tf,
                            const std::map<std::string, JointData>& joints,
                            const std::map<std::string, std::vector<std::string>>& children);

    // Geometry generators
    RenderObject createBox(float sx, float sy, float sz);
    RenderObject createCylinder(float radius, float length, int segments = 24);
    RenderObject createSphere(float radius, int rings = 12, int sectors = 18);
    RenderObject loadMesh(const std::string& uri, float sx, float sy, float sz);
    void uploadMesh(RenderObject& obj, const std::vector<Vertex>& verts);

    // Grid
    void createGrid();
    GLuint grid_vao_{0}, grid_vbo_{0};
    int grid_vertex_count_{0};

    // ROS callbacks
    void onRobotDescription(const std_msgs::msg::String::SharedPtr msg);
    void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Shader
    QOpenGLShaderProgram* shader_{nullptr};

    // Shared data (protected by mutex)
    std::mutex data_mutex_;
    std::map<std::string, std::vector<GeomDesc>> pending_geoms_;
    std::map<std::string, JointData> joints_;
    std::map<std::string, std::vector<std::string>> link_children_;
    std::string root_link_;
    bool urdf_parsed_{false};
    bool needs_rebuild_{false};

    // GL-only data (main thread only)
    std::map<std::string, LinkData> links_;

    // Camera
    float cam_yaw_{135.0f};
    float cam_pitch_{40.0f};
    float cam_distance_{1.5f};
    QVector3D cam_target_{0, 0, 0.2f};
    QPoint last_mouse_pos_;

    QTimer* render_timer_;
};

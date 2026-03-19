#include "gui/urdf_viewer.hpp"

#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <cmath>

static const char* VERT_SHADER = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix;
out vec3 FragPos;
out vec3 Normal;
void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = normalize(normalMatrix * aNormal);
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

static const char* FRAG_SHADER = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
uniform vec4 objectColor;
uniform vec3 lightDir;
out vec4 FragColor;
void main() {
    vec3 norm = normalize(Normal);
    float ambient = 0.35;
    float diff = max(dot(norm, normalize(lightDir)), 0.0);
    float back = max(dot(norm, normalize(-lightDir)), 0.0) * 0.15;
    vec3 color = (ambient + diff + back) * objectColor.rgb;
    FragColor = vec4(color, objectColor.a);
}
)";

static const char* GRID_VERT = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 view;
uniform mat4 projection;
out float dist;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    dist = length(aPos.xy);
}
)";

static const char* GRID_FRAG = R"(
#version 330 core
in float dist;
out vec4 FragColor;
void main() {
    float alpha = clamp(1.0 - dist / 3.0, 0.05, 0.3);
    FragColor = vec4(0.5, 0.5, 0.5, alpha);
}
)";

static std::string resolveUri(const std::string& uri)
{
    if (uri.size() > 10 && uri.substr(0, 10) == "package://") {
        auto rest = uri.substr(10);
        auto slash = rest.find('/');
        if (slash == std::string::npos) return "";
        auto pkg = rest.substr(0, slash);
        auto path = rest.substr(slash);
        try {
            return ament_index_cpp::get_package_share_directory(pkg) + path;
        } catch (...) {
            return "";
        }
    }
    if (uri.size() > 7 && uri.substr(0, 7) == "file://")
        return uri.substr(7);
    return uri;
}

UrdfViewer::UrdfViewer(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QOpenGLWidget(parent), node_(node)
{
    setMinimumSize(200, 200);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    desc_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        [this](std_msgs::msg::String::SharedPtr msg) { onRobotDescription(msg); });

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointStates(msg); });

    render_timer_ = new QTimer(this);
    connect(render_timer_, &QTimer::timeout, this, QOverload<>::of(&QOpenGLWidget::update));
    render_timer_->start(33); // ~30 fps
}

UrdfViewer::~UrdfViewer()
{
    makeCurrent();
    cleanupGLResources();
    if (grid_vao_) { glDeleteVertexArrays(1, &grid_vao_); glDeleteBuffers(1, &grid_vbo_); }
    delete shader_;
    doneCurrent();
}

// ── OpenGL ───────────────────────────────────────────────────────────────────

void UrdfViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.12f, 0.18f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, VERT_SHADER);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SHADER);
    shader_->link();

    createGrid();
}

void UrdfViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void UrdfViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Snapshot shared data
    bool rebuild = false;
    bool parsed = false;
    std::map<std::string, JointData> joints_snap;
    std::string root;
    std::map<std::string, std::vector<std::string>> children;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rebuild = needs_rebuild_;
        parsed = urdf_parsed_;
        if (parsed) {
            joints_snap = joints_;
            root = root_link_;
            children = link_children_;
        }
    }

    // View & projection matrices
    float yaw_r = qDegreesToRadians(cam_yaw_);
    float pitch_r = qDegreesToRadians(cam_pitch_);
    QVector3D eye(
        cam_distance_ * cosf(pitch_r) * cosf(yaw_r),
        cam_distance_ * cosf(pitch_r) * sinf(yaw_r),
        cam_distance_ * sinf(pitch_r));
    eye += cam_target_;

    QMatrix4x4 view;
    view.lookAt(eye, cam_target_, QVector3D(0, 0, 1));
    QMatrix4x4 projection;
    float aspect = width() > 0 ? float(width()) / height() : 1.0f;
    projection.perspective(45.0f, aspect, 0.01f, 100.0f);

    // Draw grid
    if (grid_vao_) {
        // Use a separate simple shader for grid — reuse main shader with identity model
        shader_->bind();
        QMatrix4x4 identity;
        shader_->setUniformValue("model", identity);
        shader_->setUniformValue("view", view);
        shader_->setUniformValue("projection", projection);
        shader_->setUniformValue("normalMatrix", identity.normalMatrix());
        shader_->setUniformValue("objectColor", QVector4D(0.4f, 0.4f, 0.4f, 0.25f));
        shader_->setUniformValue("lightDir", QVector3D(0.5f, 0.3f, 1.0f));
        glBindVertexArray(grid_vao_);
        glDrawArrays(GL_LINES, 0, grid_vertex_count_);
        shader_->release();
    }

    if (!parsed) return;

    if (rebuild) {
        buildGLResources();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            needs_rebuild_ = false;
        }
    }

    // Compute forward kinematics
    computeFKRecursive(root, QMatrix4x4(), joints_snap, children);

    // Render links
    shader_->bind();
    shader_->setUniformValue("view", view);
    shader_->setUniformValue("projection", projection);
    shader_->setUniformValue("lightDir", QVector3D(0.5f, 0.3f, 1.0f));

    for (auto& [name, ld] : links_) {
        for (auto& obj : ld.visuals) {
            if (!obj.vao || obj.vertex_count == 0) continue;
            QMatrix4x4 model = ld.world_transform * obj.visual_origin;
            shader_->setUniformValue("model", model);
            shader_->setUniformValue("normalMatrix", model.normalMatrix());
            shader_->setUniformValue("objectColor", obj.color);
            glBindVertexArray(obj.vao);
            glDrawArrays(GL_TRIANGLES, 0, obj.vertex_count);
        }
    }
    shader_->release();
}

// ── Mouse ────────────────────────────────────────────────────────────────────

void UrdfViewer::mousePressEvent(QMouseEvent* e)
{
    last_mouse_pos_ = e->pos();
}

void UrdfViewer::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - last_mouse_pos_;
    if (e->buttons() & Qt::LeftButton) {
        cam_yaw_ -= delta.x() * 0.4f;
        cam_pitch_ += delta.y() * 0.4f;
        cam_pitch_ = qBound(-89.0f, cam_pitch_, 89.0f);
    } else if (e->buttons() & Qt::MiddleButton) {
        float yaw_r = qDegreesToRadians(cam_yaw_);
        QVector3D right(-sinf(yaw_r), cosf(yaw_r), 0);
        QVector3D up(0, 0, 1);
        cam_target_ -= right * delta.x() * 0.002f * cam_distance_;
        cam_target_ += up * delta.y() * 0.002f * cam_distance_;
    }
    last_mouse_pos_ = e->pos();
    update();
}

void UrdfViewer::wheelEvent(QWheelEvent* e)
{
    cam_distance_ *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    cam_distance_ = qBound(0.05f, cam_distance_, 50.0f);
    update();
}

// ── URDF parsing (ROS thread) ───────────────────────────────────────────────

void UrdfViewer::onRobotDescription(const std_msgs::msg::String::SharedPtr msg)
{
    parseUrdf(msg->data);
}

void UrdfViewer::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        auto it = joints_.find(msg->name[i]);
        if (it != joints_.end())
            it->second.value = msg->position[i];
    }
}

void UrdfViewer::parseUrdf(const std::string& urdf_xml)
{
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF");
        return;
    }

    std::map<std::string, std::vector<GeomDesc>> geoms;
    std::map<std::string, JointData> jdata;
    std::map<std::string, std::vector<std::string>> children;

    // Process links
    for (const auto& [name, link] : model.links_) {
        std::vector<GeomDesc> link_geoms;

        // Gather visuals — use visual_array, fall back to single visual
        std::vector<urdf::VisualSharedPtr> vis_list = link->visual_array;
        if (vis_list.empty() && link->visual)
            vis_list.push_back(link->visual);

        for (const auto& vis : vis_list) {
            if (!vis || !vis->geometry) continue;
            GeomDesc gd;

            // Origin
            auto& o = vis->origin;
            double r, p, y;
            o.rotation.getRPY(r, p, y);
            gd.visual_origin.translate(
                static_cast<float>(o.position.x),
                static_cast<float>(o.position.y),
                static_cast<float>(o.position.z));
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(y)), 0, 0, 1);
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(p)), 0, 1, 0);
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(r)), 1, 0, 0);

            // Color
            if (vis->material) {
                gd.color = QVector4D(
                    static_cast<float>(vis->material->color.r),
                    static_cast<float>(vis->material->color.g),
                    static_cast<float>(vis->material->color.b),
                    static_cast<float>(vis->material->color.a));
            }

            // Geometry
            auto geom = vis->geometry;
            switch (geom->type) {
            case urdf::Geometry::BOX: {
                auto box = std::static_pointer_cast<urdf::Box>(geom);
                gd.type = GeomDesc::BOX;
                gd.dims[0] = static_cast<float>(box->dim.x);
                gd.dims[1] = static_cast<float>(box->dim.y);
                gd.dims[2] = static_cast<float>(box->dim.z);
                break;
            }
            case urdf::Geometry::CYLINDER: {
                auto cyl = std::static_pointer_cast<urdf::Cylinder>(geom);
                gd.type = GeomDesc::CYLINDER;
                gd.dims[0] = static_cast<float>(cyl->radius);
                gd.dims[1] = static_cast<float>(cyl->length);
                break;
            }
            case urdf::Geometry::SPHERE: {
                auto sph = std::static_pointer_cast<urdf::Sphere>(geom);
                gd.type = GeomDesc::SPHERE;
                gd.dims[0] = static_cast<float>(sph->radius);
                break;
            }
            case urdf::Geometry::MESH: {
                auto mesh = std::static_pointer_cast<urdf::Mesh>(geom);
                gd.type = GeomDesc::MESH;
                gd.mesh_uri = mesh->filename;
                gd.dims[0] = static_cast<float>(mesh->scale.x);
                gd.dims[1] = static_cast<float>(mesh->scale.y);
                gd.dims[2] = static_cast<float>(mesh->scale.z);
                break;
            }
            default: continue;
            }
            link_geoms.push_back(std::move(gd));
        }
        geoms[name] = std::move(link_geoms);
    }

    // Process joints
    for (const auto& [name, joint] : model.joints_) {
        JointData jd;
        jd.parent_link = joint->parent_link_name;
        jd.child_link = joint->child_link_name;
        jd.type = joint->type;

        auto& o = joint->parent_to_joint_origin_transform;
        double r, p, y;
        o.rotation.getRPY(r, p, y);
        jd.origin.translate(
            static_cast<float>(o.position.x),
            static_cast<float>(o.position.y),
            static_cast<float>(o.position.z));
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(y)), 0, 0, 1);
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(p)), 0, 1, 0);
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(r)), 1, 0, 0);

        jd.axis = QVector3D(
            static_cast<float>(joint->axis.x),
            static_cast<float>(joint->axis.y),
            static_cast<float>(joint->axis.z));

        jdata[name] = std::move(jd);
        children[joint->parent_link_name].push_back(name);
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_geoms_ = std::move(geoms);
        joints_ = std::move(jdata);
        link_children_ = std::move(children);
        root_link_ = model.getRoot()->name;
        urdf_parsed_ = true;
        needs_rebuild_ = true;
    }

    RCLCPP_INFO(node_->get_logger(), "URDF loaded: %s (%zu links, %zu joints)",
                model.getName().c_str(), model.links_.size(), model.joints_.size());
}

// ── GL resource building (main thread) ──────────────────────────────────────

void UrdfViewer::buildGLResources()
{
    std::map<std::string, std::vector<GeomDesc>> geom_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        geom_copy = std::move(pending_geoms_);
    }

    cleanupGLResources();

    for (auto& [link_name, descs] : geom_copy) {
        LinkData ld;
        for (auto& gd : descs) {
            RenderObject obj;
            switch (gd.type) {
            case GeomDesc::BOX:
                obj = createBox(gd.dims[0], gd.dims[1], gd.dims[2]);
                break;
            case GeomDesc::CYLINDER:
                obj = createCylinder(gd.dims[0], gd.dims[1]);
                break;
            case GeomDesc::SPHERE:
                obj = createSphere(gd.dims[0]);
                break;
            case GeomDesc::MESH:
                obj = loadMesh(gd.mesh_uri, gd.dims[0], gd.dims[1], gd.dims[2]);
                break;
            }
            obj.color = gd.color;
            obj.visual_origin = gd.visual_origin;
            ld.visuals.push_back(std::move(obj));
        }
        links_[link_name] = std::move(ld);
    }
}

void UrdfViewer::cleanupGLResources()
{
    for (auto& [name, ld] : links_) {
        for (auto& obj : ld.visuals) {
            if (obj.vao) glDeleteVertexArrays(1, &obj.vao);
            if (obj.vbo) glDeleteBuffers(1, &obj.vbo);
        }
    }
    links_.clear();
}

// ── Forward kinematics ──────────────────────────────────────────────────────

void UrdfViewer::computeFKRecursive(
    const std::string& link_name,
    const QMatrix4x4& parent_tf,
    const std::map<std::string, JointData>& joints,
    const std::map<std::string, std::vector<std::string>>& children)
{
    auto link_it = links_.find(link_name);
    if (link_it != links_.end())
        link_it->second.world_transform = parent_tf;

    auto ch_it = children.find(link_name);
    if (ch_it == children.end()) return;

    for (const auto& joint_name : ch_it->second) {
        auto jt = joints.find(joint_name);
        if (jt == joints.end()) continue;

        const auto& jd = jt->second;
        QMatrix4x4 tf = parent_tf * jd.origin;

        // Apply joint value
        if (jd.type == 1 || jd.type == 2) { // REVOLUTE or CONTINUOUS
            QMatrix4x4 rot;
            rot.rotate(static_cast<float>(qRadiansToDegrees(jd.value)), jd.axis);
            tf *= rot;
        } else if (jd.type == 3) { // PRISMATIC
            QMatrix4x4 trans;
            trans.translate(jd.axis * static_cast<float>(jd.value));
            tf *= trans;
        }

        computeFKRecursive(jd.child_link, tf, joints, children);
    }
}

// ── Geometry generators ─────────────────────────────────────────────────────

void UrdfViewer::uploadMesh(RenderObject& obj, const std::vector<Vertex>& verts)
{
    obj.vertex_count = static_cast<int>(verts.size());
    glGenVertexArrays(1, &obj.vao);
    glGenBuffers(1, &obj.vbo);
    glBindVertexArray(obj.vao);
    glBindBuffer(GL_ARRAY_BUFFER, obj.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(Vertex)),
                 verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, pos)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

UrdfViewer::RenderObject UrdfViewer::createBox(float sx, float sy, float sz)
{
    float hx = sx * 0.5f, hy = sy * 0.5f, hz = sz * 0.5f;
    // clang-format off
    std::vector<Vertex> v;
    auto quad = [&](float nx, float ny, float nz,
                    float ax, float ay, float az,
                    float bx, float by, float bz,
                    float cx, float cy, float cz,
                    float dx, float dy, float dz) {
        Vertex va{{ax,ay,az},{nx,ny,nz}}, vb{{bx,by,bz},{nx,ny,nz}};
        Vertex vc{{cx,cy,cz},{nx,ny,nz}}, vd{{dx,dy,dz},{nx,ny,nz}};
        v.insert(v.end(), {va, vb, vc, vc, vd, va});
    };
    quad( 0, 0, 1, -hx,-hy, hz,  hx,-hy, hz,  hx, hy, hz, -hx, hy, hz);
    quad( 0, 0,-1, -hx, hy,-hz,  hx, hy,-hz,  hx,-hy,-hz, -hx,-hy,-hz);
    quad( 1, 0, 0,  hx,-hy,-hz,  hx, hy,-hz,  hx, hy, hz,  hx,-hy, hz);
    quad(-1, 0, 0, -hx,-hy, hz, -hx, hy, hz, -hx, hy,-hz, -hx,-hy,-hz);
    quad( 0, 1, 0, -hx, hy, hz,  hx, hy, hz,  hx, hy,-hz, -hx, hy,-hz);
    quad( 0,-1, 0, -hx,-hy,-hz,  hx,-hy,-hz,  hx,-hy, hz, -hx,-hy, hz);
    // clang-format on
    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::createCylinder(float radius, float length, int seg)
{
    float hz = length * 0.5f;
    std::vector<Vertex> v;

    for (int i = 0; i < seg; ++i) {
        float a0 = 2.0f * M_PI * i / seg;
        float a1 = 2.0f * M_PI * (i + 1) / seg;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float x0 = radius * c0, y0 = radius * s0;
        float x1 = radius * c1, y1 = radius * s1;

        // Side
        Vertex sa{{x0,y0, hz},{c0,s0,0}}, sb{{x1,y1, hz},{c1,s1,0}};
        Vertex sc{{x1,y1,-hz},{c1,s1,0}}, sd{{x0,y0,-hz},{c0,s0,0}};
        v.insert(v.end(), {sa, sb, sc, sc, sd, sa});

        // Top cap
        Vertex tc{{0,0,hz},{0,0,1}}, ta{{x0,y0,hz},{0,0,1}}, tb{{x1,y1,hz},{0,0,1}};
        v.insert(v.end(), {tc, ta, tb});

        // Bottom cap
        Vertex bc{{0,0,-hz},{0,0,-1}}, ba{{x1,y1,-hz},{0,0,-1}}, bb{{x0,y0,-hz},{0,0,-1}};
        v.insert(v.end(), {bc, ba, bb});
    }

    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::createSphere(float radius, int rings, int sectors)
{
    std::vector<Vertex> v;
    for (int r = 0; r < rings; ++r) {
        float phi0 = M_PI * r / rings - M_PI / 2;
        float phi1 = M_PI * (r + 1) / rings - M_PI / 2;
        for (int s = 0; s < sectors; ++s) {
            float th0 = 2.0f * M_PI * s / sectors;
            float th1 = 2.0f * M_PI * (s + 1) / sectors;

            auto pt = [&](float phi, float th) -> Vertex {
                float cp = cosf(phi), sp = sinf(phi);
                float ct = cosf(th), st = sinf(th);
                float x = cp * ct, y = cp * st, z = sp;
                return {{radius*x, radius*y, radius*z}, {x, y, z}};
            };

            Vertex a = pt(phi0, th0), b = pt(phi0, th1);
            Vertex c = pt(phi1, th1), d = pt(phi1, th0);
            v.insert(v.end(), {a, b, c, c, d, a});
        }
    }

    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::loadMesh(const std::string& uri,
                                               float sx, float sy, float sz)
{
    std::string path = resolveUri(uri);
    if (path.empty())
        return createBox(0.05f, 0.05f, 0.05f);

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || !scene->mRootNode || scene->mNumMeshes == 0) {
        RCLCPP_WARN(node_->get_logger(), "Could not load mesh: %s", path.c_str());
        return createBox(0.05f, 0.05f, 0.05f);
    }

    std::vector<Vertex> verts;
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            for (unsigned int i = 0; i < face.mNumIndices; ++i) {
                unsigned int idx = face.mIndices[i];
                Vertex v;
                v.pos[0] = mesh->mVertices[idx].x * sx;
                v.pos[1] = mesh->mVertices[idx].y * sy;
                v.pos[2] = mesh->mVertices[idx].z * sz;
                if (mesh->HasNormals()) {
                    v.normal[0] = mesh->mNormals[idx].x;
                    v.normal[1] = mesh->mNormals[idx].y;
                    v.normal[2] = mesh->mNormals[idx].z;
                } else {
                    v.normal[0] = 0; v.normal[1] = 0; v.normal[2] = 1;
                }
                verts.push_back(v);
            }
        }
    }

    RenderObject obj;
    uploadMesh(obj, verts);
    return obj;
}

// ── Grid ─────────────────────────────────────────────────────────────────────

void UrdfViewer::createGrid()
{
    std::vector<Vertex> v;
    float extent = 3.0f;
    float step = 0.25f;
    Vertex zero{};
    for (float i = -extent; i <= extent; i += step) {
        Vertex a = zero, b = zero, c = zero, d = zero;
        a.pos[0] = i; a.pos[1] = -extent;
        b.pos[0] = i; b.pos[1] =  extent;
        c.pos[0] = -extent; c.pos[1] = i;
        d.pos[0] =  extent; d.pos[1] = i;
        v.insert(v.end(), {a, b, c, d});
    }
    grid_vertex_count_ = static_cast<int>(v.size());

    glGenVertexArrays(1, &grid_vao_);
    glGenBuffers(1, &grid_vbo_);
    glBindVertexArray(grid_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, grid_vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(v.size() * sizeof(Vertex)),
                 v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, pos)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

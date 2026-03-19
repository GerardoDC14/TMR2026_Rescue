#include "gui/source_manager.hpp"
#include "gui/camera_hub.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include <opencv2/opencv.hpp>

#include <algorithm>

SourceManager::SourceManager(rclcpp::Node::SharedPtr node,
                             std::shared_ptr<CameraHub> hub,
                             QObject* parent)
    : QObject(parent), node_(node), camera_hub_(hub)
{
    // Subscribe to /config with transient local QoS to get latched config
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    config_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/config", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onConfigReceived(msg);
        });
}

SourceManager::~SourceManager()
{
    if (probe_thread_.joinable())
        probe_thread_.join();
}

void SourceManager::discoverSources()
{
    if (probing_.exchange(true))
        return; // already probing

    // Join any previous probe thread
    if (probe_thread_.joinable())
        probe_thread_.join();

    probe_thread_ = std::thread([this]() {
        probeLocalCameras();
        rebuildSourceList();
        probing_ = false;
        emit sourcesUpdated();
    });
}

QStringList SourceManager::sourceNames() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return source_names_;
}

QStringList SourceManager::sourceIdentifiers() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return source_ids_;
}

void SourceManager::probeLocalCameras()
{
    std::vector<int> found;

    // Cameras already streaming in the hub can't be re-opened for probing,
    // so include them directly.
    std::vector<int> active;
    if (camera_hub_)
        active = camera_hub_->activeCameraIds();

    for (int i = 0; i < 10; ++i) {
        if (std::find(active.begin(), active.end(), i) != active.end()) {
            found.push_back(i);
            continue;
        }
        cv::VideoCapture cap;
        if (cap.open(i, cv::CAP_V4L2)) {
            if (cap.isOpened()) {
                found.push_back(i);
                cap.release();
            }
        }
    }

    std::lock_guard<std::mutex> lock(local_mutex_);
    local_camera_ids_ = std::move(found);
}

void SourceManager::onConfigReceived(const std_msgs::msg::String::SharedPtr msg)
{
    // Parse JSON config: { "camera_topics": ["/cam1/image_raw", ...], ... }
    QJsonDocument doc = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data));

    std::vector<std::string> cam_topics, therm_topics;
    if (doc.isObject()) {
        QJsonArray cam_arr = doc.object().value("camera_topics").toArray();
        for (const auto& val : cam_arr)
            if (val.isString())
                cam_topics.push_back(val.toString().toStdString());

        QJsonArray therm_arr = doc.object().value("thermal_topics").toArray();
        for (const auto& val : therm_arr)
            if (val.isString())
                therm_topics.push_back(val.toString().toStdString());
    }

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_topics_ = std::move(cam_topics);
        thermal_topics_ = std::move(therm_topics);
    }

    rebuildSourceList();
    emit sourcesUpdated();
}

void SourceManager::rebuildSourceList()
{
    QStringList names, ids;

    // Add "None" option
    names << "None";
    ids << "";

    // Local cameras
    {
        std::lock_guard<std::mutex> lock(local_mutex_);
        for (int id : local_camera_ids_) {
            names << QString("Camera %1").arg(id);
            ids << QString("local:%1").arg(id);
        }
    }

    // ROS topic cameras
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        for (const auto& topic : config_topics_) {
            QString qtopic = QString::fromStdString(topic);
            names << qtopic;
            ids << QString("topic:") + qtopic;
        }
        // Thermal cameras
        for (const auto& topic : thermal_topics_) {
            QString qtopic = QString::fromStdString(topic);
            names << "Thermal: " + qtopic;
            ids << "thermal:" + qtopic;
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    source_names_ = std::move(names);
    source_ids_ = std::move(ids);
}

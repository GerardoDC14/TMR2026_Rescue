#pragma once

#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QMouseEvent>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class CameraHub;
struct FilterConfig;

class ClickableLabel : public QLabel {
    Q_OBJECT
public:
    using QLabel::QLabel;
signals:
    void clicked();
protected:
    void mousePressEvent(QMouseEvent*) override { emit clicked(); }
};

class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(rclcpp::Node::SharedPtr node,
                         std::shared_ptr<CameraHub> hub,
                         QWidget* parent = nullptr);
    ~VideoWidget() override;

    void setPaused(bool paused);
    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

signals:
    void displayClicked();
    void frameReady(const QImage& image);
    void thermalActiveChanged(bool active);

private slots:
    void onSourceChanged(int index);
    void onFilterChanged(int index);
    void onFrameReady(const QImage& image);

private:
    enum class SourceType { NONE, LOCAL, ROS_TOPIC, THERMAL };

    void workerLoop();
    void onImageReceived(const sensor_msgs::msg::Image::SharedPtr& msg);
    void onThermalReceived(const sensor_msgs::msg::Image::SharedPtr& msg);
    static QImage matToQImage(const cv::Mat& mat);

    void setupFilterOptions(const std::string& filter_name);
    void clearFilterOptions();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<CameraHub> camera_hub_;

    QComboBox* source_combo_;
    QComboBox* filter_combo_;
    ClickableLabel* display_;

    // Image controls (always visible, between display and filter options)
    QWidget* image_controls_container_{nullptr};
    std::atomic<int> brightness_{0};      // -100..+100, default 0
    std::atomic<int> contrast_x100_{100}; // 50..300 → 0.5x..3.0x, default 1.0x

    // Filter options (dynamic, below video)
    QWidget* options_container_{nullptr};

    // Worker thread
    std::thread worker_;
    std::atomic<bool> running_{true};
    std::atomic<bool> paused_{false};
    std::atomic<SourceType> source_type_{SourceType::NONE};
    std::atomic<int> current_local_id_{-1};

    // ROS subscriptions
    std::mutex sub_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;

    // Latest frame from ROS callback
    std::mutex frame_mutex_;
    cv::Mat latest_ros_frame_;

    // Current filter (per-widget instance + config)
    std::mutex filter_mutex_;
    std::function<cv::Mat(const cv::Mat&)> current_filter_func_;
    std::shared_ptr<FilterConfig> filter_config_;
};

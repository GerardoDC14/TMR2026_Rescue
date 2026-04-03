#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>

class CameraHub;
class VideoWidget;
class OdometryPanel;

class VideoPanel : public QWidget {
    Q_OBJECT
public:
    explicit VideoPanel(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<CameraHub> hub,
                        QWidget* parent = nullptr);

    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

    OdometryPanel* odometryPanel() const { return odom_panel_; }

signals:
    // Emitted when the number of widgets showing the thermal source transitions
    // between zero and non-zero (active=true when first widget selects thermal,
    // active=false when last widget deselects it).
    void thermalActiveChanged(bool active);

private slots:
    void onWidgetClicked(int index);
    void onWidgetThermalChanged(bool active);

private:
    QGridLayout* grid_;
    VideoWidget* widgets_[3];       // indices 0-2 (top-left, top-right, bottom-left)
    OdometryPanel* odom_panel_;     // bottom-right slot
    int enlarged_index_{-1};
    std::atomic<int> thermal_count_{0};  // how many widgets currently show thermal
};

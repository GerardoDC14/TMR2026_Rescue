#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>

#include <memory>

class CameraHub;
class VideoWidget;

class VideoPanel : public QWidget {
    Q_OBJECT
public:
    explicit VideoPanel(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<CameraHub> hub,
                        QWidget* parent = nullptr);

    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

private slots:
    void onWidgetClicked(int index);

private:
    QGridLayout* grid_;
    VideoWidget* widgets_[4];
    int enlarged_index_{-1};
};

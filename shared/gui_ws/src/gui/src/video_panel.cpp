#include "gui/video_panel.hpp"
#include "gui/video_widget.hpp"

VideoPanel::VideoPanel(rclcpp::Node::SharedPtr node,
                       std::shared_ptr<CameraHub> hub,
                       QWidget* parent)
    : QWidget(parent)
{
    grid_ = new QGridLayout(this);
    grid_->setContentsMargins(0, 0, 0, 0);
    grid_->setSpacing(2);

    for (int i = 0; i < 4; ++i) {
        widgets_[i] = new VideoWidget(node, hub, this);
        grid_->addWidget(widgets_[i], i / 2, i % 2);

        // Connect click signal with captured index
        connect(widgets_[i], &VideoWidget::displayClicked, this, [this, i]() {
            onWidgetClicked(i);
        });
    }
}

void VideoPanel::updateSources(const QStringList& names, const QStringList& identifiers)
{
    for (int i = 0; i < 4; ++i)
        widgets_[i]->updateSources(names, identifiers);
}

void VideoPanel::updateFilters(const QStringList& names)
{
    for (int i = 0; i < 4; ++i)
        widgets_[i]->updateFilters(names);
}

void VideoPanel::onWidgetClicked(int index)
{
    if (enlarged_index_ == index) {
        // Restore 2x2 layout
        for (int i = 0; i < 4; ++i) {
            widgets_[i]->setPaused(false);
            widgets_[i]->show();
        }
        enlarged_index_ = -1;
    } else {
        // Enlarge clicked widget, pause others
        for (int i = 0; i < 4; ++i) {
            if (i == index) {
                widgets_[i]->setPaused(false);
                widgets_[i]->show();
            } else {
                widgets_[i]->setPaused(true);
                widgets_[i]->hide();
            }
        }
        enlarged_index_ = index;
    }
}

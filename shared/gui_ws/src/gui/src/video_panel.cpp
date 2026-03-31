#include "gui/video_panel.hpp"
#include "gui/video_widget.hpp"
#include "gui/odometry_panel.hpp"

// Grid positions for the 3 video widgets + odometry panel
static constexpr int POS[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};

VideoPanel::VideoPanel(rclcpp::Node::SharedPtr node,
                       std::shared_ptr<CameraHub> hub,
                       QWidget* parent)
    : QWidget(parent)
{
    grid_ = new QGridLayout(this);
    grid_->setContentsMargins(0, 0, 0, 0);
    grid_->setSpacing(2);

    // 3 video widgets: top-left (0), top-right (1), bottom-left (2)
    for (int i = 0; i < 3; ++i) {
        widgets_[i] = new VideoWidget(node, hub, this);
        grid_->addWidget(widgets_[i], POS[i][0], POS[i][1]);

        connect(widgets_[i], &VideoWidget::displayClicked, this, [this, i]() {
            onWidgetClicked(i);
        });
    }

    // Odometry panel in bottom-right (1, 1)
    odom_panel_ = new OdometryPanel(node, this);
    grid_->addWidget(odom_panel_, POS[3][0], POS[3][1]);

    // Equal row/column stretch so all four cells stay the same size
    grid_->setRowStretch(0, 1);
    grid_->setRowStretch(1, 1);
    grid_->setColumnStretch(0, 1);
    grid_->setColumnStretch(1, 1);
}

void VideoPanel::updateSources(const QStringList& names, const QStringList& identifiers)
{
    for (int i = 0; i < 3; ++i)
        widgets_[i]->updateSources(names, identifiers);
}

void VideoPanel::updateFilters(const QStringList& names)
{
    for (int i = 0; i < 3; ++i)
        widgets_[i]->updateFilters(names);
}

void VideoPanel::onWidgetClicked(int index)
{
    if (enlarged_index_ == index) {
        // ── Restore 2x2 layout ──────────────────────────────────────────────

        // Remove the enlarged widget from the grid (it currently spans 2x2)
        grid_->removeWidget(widgets_[index]);

        // Re-add all widgets in their original 1x1 positions
        for (int i = 0; i < 3; ++i) {
            grid_->addWidget(widgets_[i], POS[i][0], POS[i][1]);
            widgets_[i]->setPaused(false);
            widgets_[i]->show();
        }
        grid_->addWidget(odom_panel_, POS[3][0], POS[3][1]);
        odom_panel_->show();

        grid_->setRowStretch(0, 1);
        grid_->setRowStretch(1, 1);
        grid_->setColumnStretch(0, 1);
        grid_->setColumnStretch(1, 1);

        enlarged_index_ = -1;
    } else {
        // ── Enlarge clicked widget to span 2x2 ─────────────────────────────

        // Hide + pause non-selected video widgets and the odometry panel
        for (int i = 0; i < 3; ++i) {
            if (i != index) {
                widgets_[i]->setPaused(true);
                widgets_[i]->hide();
            }
        }
        odom_panel_->hide();

        // Remove the selected widget from its current cell and re-add spanning 2x2
        grid_->removeWidget(widgets_[index]);
        grid_->addWidget(widgets_[index], 0, 0, 2, 2);
        widgets_[index]->setPaused(false);
        widgets_[index]->show();

        enlarged_index_ = index;
    }
}

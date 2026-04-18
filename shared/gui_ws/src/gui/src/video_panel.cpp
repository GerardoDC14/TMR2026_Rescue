#include "gui/video_panel.hpp"
#include "gui/video_widget.hpp"
#include "gui/odometry_panel.hpp"

// 2 rows x 3 cols grid positions, row-major
static constexpr int POS[6][2] = {
    {0, 0}, {0, 1}, {0, 2},
    {1, 0}, {1, 1}, {1, 2},
};
static constexpr int kNumWidgets = 6;
static constexpr int kRows = 2;
static constexpr int kCols = 3;

VideoPanel::VideoPanel(rclcpp::Node::SharedPtr node,
                       std::shared_ptr<CameraHub> hub,
                       QWidget* parent)
    : QWidget(parent)
{
    grid_ = new QGridLayout(this);
    grid_->setContentsMargins(0, 0, 0, 0);
    grid_->setSpacing(2);

    for (int i = 0; i < kNumWidgets; ++i) {
        widgets_[i] = new VideoWidget(node, hub, this);
        grid_->addWidget(widgets_[i], POS[i][0], POS[i][1]);

        connect(widgets_[i], &VideoWidget::displayClicked, this, [this, i]() {
            onWidgetClicked(i);
        });
        connect(widgets_[i], &VideoWidget::thermalActiveChanged,
                this, &VideoPanel::onWidgetThermalChanged);
    }

    // Odometry panel kept alive for setRobotType() callers, but not shown.
    // Parent to `this` so it is destroyed with the panel; hidden so it never
    // takes layout space.
    odom_panel_ = new OdometryPanel(node, this);
    odom_panel_->hide();

    // Equal row/column stretch so all cells stay the same size
    for (int r = 0; r < kRows; ++r) grid_->setRowStretch(r, 1);
    for (int c = 0; c < kCols; ++c) grid_->setColumnStretch(c, 1);
}

void VideoPanel::updateSources(const QStringList& names, const QStringList& identifiers)
{
    for (int i = 0; i < kNumWidgets; ++i)
        widgets_[i]->updateSources(names, identifiers);
}

void VideoPanel::updateFilters(const QStringList& names)
{
    for (int i = 0; i < kNumWidgets; ++i)
        widgets_[i]->updateFilters(names);
}

void VideoPanel::onWidgetThermalChanged(bool active)
{
    int prev = thermal_count_.load();
    int next = active ? prev + 1 : std::max(0, prev - 1);
    thermal_count_ = next;

    // Emit on transitions: 0→1 (any widget picked thermal) or N→0 (last one dropped it)
    if (prev == 0 && next == 1)
        emit thermalActiveChanged(true);
    else if (prev > 0 && next == 0)
        emit thermalActiveChanged(false);
}

void VideoPanel::onWidgetClicked(int index)
{
    if (enlarged_index_ == index) {
        // ── Restore grid layout ─────────────────────────────────────────────
        grid_->removeWidget(widgets_[index]);

        for (int i = 0; i < kNumWidgets; ++i) {
            grid_->addWidget(widgets_[i], POS[i][0], POS[i][1]);
            widgets_[i]->setPaused(false);
            widgets_[i]->show();
        }

        for (int r = 0; r < kRows; ++r) grid_->setRowStretch(r, 1);
        for (int c = 0; c < kCols; ++c) grid_->setColumnStretch(c, 1);

        enlarged_index_ = -1;
    } else {
        // ── Enlarge clicked widget to span the full grid ────────────────────
        for (int i = 0; i < kNumWidgets; ++i) {
            if (i != index) {
                widgets_[i]->setPaused(true);
                widgets_[i]->hide();
            }
        }

        grid_->removeWidget(widgets_[index]);
        grid_->addWidget(widgets_[index], 0, 0, kRows, kCols);
        widgets_[index]->setPaused(false);
        widgets_[index]->show();

        enlarged_index_ = index;
    }
}

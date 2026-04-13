#include "gui/ppm_calib_dialog.hpp"
#include "gui/app_settings.hpp"

#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

const char* PpmCalibDialog::CH_NAMES[6] = {
    "Ch1", "Ch2", "Ch3", "Ch4", "Ch5", "Ch6"
};

// ── Constructor ───────────────────────────────────────────────────────────────

PpmCalibDialog::PpmCalibDialog(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QDialog(parent), node_(node)
{
    setWindowTitle("RC Channel Calibration");
    setMinimumWidth(680);
    setStyleSheet(
        "QDialog { background-color: #1e1e2e; }"
        "QGroupBox { color: #aaa; border: 1px solid #3a3a55; border-radius: 4px; "
        "           margin-top: 8px; padding: 8px 6px 6px 6px; font-size: 11px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #4fc3f7; }"
        "QLabel { color: #aaa; font-size: 11px; }"
        "QSpinBox { background: #2d2d45; color: #e0e0e0; border: 1px solid #3a3a55; "
        "           border-radius: 3px; padding: 2px 4px; font-size: 11px; min-width: 60px; }"
        "QSpinBox::up-button, QSpinBox::down-button { border: none; width: 14px; }"
        "QPushButton { background-color: #2d2d45; color: #e0e0e0; padding: 4px 10px; "
        "              border: 1px solid #3a3a55; border-radius: 3px; font-size: 11px; }"
        "QPushButton:hover { background-color: #3a3a55; }"
        "QProgressBar { border: 1px solid #3a3a55; border-radius: 3px; "
        "               background: #2d2d45; height: 14px; text-align: center; "
        "               color: #888; font-size: 10px; }"
        "QProgressBar::chunk { background: qlineargradient(x1:0,y1:0,x2:1,y2:0,"
        "   stop:0 #1a6a8a, stop:0.5 #4fc3f7, stop:1 #1a6a8a); border-radius: 2px; }"
    );

    // Connect internal signal for thread-safe ROS → GUI updates
    connect(this, &PpmCalibDialog::ppmReceived,
            this, &PpmCalibDialog::onPpmReceived,
            Qt::QueuedConnection);

    auto* root = new QVBoxLayout(this);
    root->setSpacing(8);
    root->setContentsMargins(12, 12, 12, 12);

    auto* info = new QLabel(
        "Move each stick/switch to its extremes and centre, then use Capture or type values.\n"
        "The live bar shows the current position within the calibrated range.", this);
    info->setWordWrap(true);
    info->setStyleSheet("color: #888; font-size: 10px;");
    root->addWidget(info);

    // ── Channel grid ─────────────────────────────────────────────────────────
    auto* grid_group = new QGroupBox("Channel Calibration", this);
    auto* grid = new QGridLayout(grid_group);
    grid->setSpacing(5);
    grid->setContentsMargins(6, 6, 6, 8);

    auto make_hdr = [&](const char* txt, int col) {
        auto* l = new QLabel(txt, grid_group);
        l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        l->setStyleSheet("color: #4fc3f7; font-weight: bold; font-size: 11px;");
        grid->addWidget(l, 0, col);
    };
    make_hdr("Ch",      0);
    make_hdr("Live position",  1);
    make_hdr("µs",      2);
    make_hdr("Min",     3);
    make_hdr("Center",  4);
    make_hdr("Max",     5);
    make_hdr("",        6);

    auto make_spin = [&](QWidget* parent) {
        auto* s = new QSpinBox(parent);
        s->setRange(950, 2050);
        s->setSingleStep(5);
        s->setSuffix(" µs");
        return s;
    };

    for (int c = 0; c < 6; ++c) {
        // Channel name
        auto* ch_lbl = new QLabel(CH_NAMES[c], grid_group);
        ch_lbl->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        ch_lbl->setStyleSheet("color: #ccc; font-weight: bold;");
        grid->addWidget(ch_lbl, c + 1, 0);

        // Live bar: range is raw 950..2050; value updated from /robot/ppm
        bars_[c] = new QProgressBar(grid_group);
        bars_[c]->setRange(950, 2050);
        bars_[c]->setValue(1500);
        bars_[c]->setFormat("");      // no text inside bar
        bars_[c]->setMinimumWidth(180);
        grid->addWidget(bars_[c], c + 1, 1);

        // Numeric value label
        val_labels_[c] = new QLabel("1500 µs", grid_group);
        val_labels_[c]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        val_labels_[c]->setStyleSheet("color: #4fc3f7; min-width: 55px; font-size: 11px;");
        grid->addWidget(val_labels_[c], c + 1, 2);

        // Min / Center / Max spinboxes
        min_spins_[c]     = make_spin(grid_group);
        neutral_spins_[c] = make_spin(grid_group);
        max_spins_[c]     = make_spin(grid_group);
        min_spins_[c]    ->setValue(1000);
        neutral_spins_[c]->setValue(1500);
        max_spins_[c]    ->setValue(2000);
        grid->addWidget(min_spins_[c],     c + 1, 3);
        grid->addWidget(neutral_spins_[c], c + 1, 4);
        grid->addWidget(max_spins_[c],     c + 1, 5);

        // "Capture Center" button — stores current live value into neutral spin
        auto* cap_btn = new QPushButton("◎ Capture Center", grid_group);
        cap_btn->setToolTip("Set Center to the channel's current raw PPM value");
        cap_btn->setStyleSheet(
            "QPushButton { background-color: #2a4a2a; border-color: #3a6a3a; font-size: 10px; }"
            "QPushButton:hover { background-color: #3a6a3a; }");
        const int ch = c;
        connect(cap_btn, &QPushButton::clicked, [this, ch]() { onCaptureCenter(ch); });
        grid->addWidget(cap_btn, c + 1, 6);
    }

    // ── Deadband row (below all channels) ────────────────────────────────────
    {
        auto* sep = new QFrame(grid_group);
        sep->setFrameShape(QFrame::HLine);
        sep->setStyleSheet("color: #3a3a55;");
        grid->addWidget(sep, 7, 0, 1, 7);

        auto* db_lbl = new QLabel("Stick Deadband", grid_group);
        db_lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        db_lbl->setStyleSheet("color: #4fc3f7; font-weight: bold;");
        grid->addWidget(db_lbl, 8, 0, 1, 3);

        deadband_spin_ = new QDoubleSpinBox(grid_group);
        deadband_spin_->setRange(0.0, 30.0);
        deadband_spin_->setSingleStep(0.5);
        deadband_spin_->setDecimals(1);
        deadband_spin_->setSuffix(" %");
        deadband_spin_->setValue(5.0);
        deadband_spin_->setToolTip(
            "Normalised dead zone around stick centre (percent of full range).\n"
            "Values below this threshold are treated as zero.");
        grid->addWidget(deadband_spin_, 8, 3, 1, 2);
    }

    grid->setColumnStretch(1, 1);   // bar column stretches
    root->addWidget(grid_group);

    // ── Buttons ───────────────────────────────────────────────────────────────
    root->addStretch();
    auto* sep = new QFrame(this);
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color: #3a3a55;");
    root->addWidget(sep);

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();

    auto* apply_btn = new QPushButton("Apply", this);
    apply_btn->setStyleSheet(
        "QPushButton { background-color: #1a4a8a; border-color: #2a5a9a; }"
        "QPushButton:hover { background-color: #2a5a9a; }");
    connect(apply_btn, &QPushButton::clicked, this, &PpmCalibDialog::onApply);

    auto* close_btn = new QPushButton("Close", this);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::accept);

    btn_row->addWidget(apply_btn);
    btn_row->addWidget(close_btn);
    root->addLayout(btn_row);

    // ── ROS subscription ─────────────────────────────────────────────────────
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    ppm_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/robot/ppm", qos,
        [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
            QVector<int> v;
            v.reserve(6);
            for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); ++i)
                v.append(msg->data[i]);
            emit ppmReceived(v);
        });
}

PpmCalibDialog::~PpmCalibDialog()
{
    ppm_sub_.reset();
}

// ── Sync ↔ AppSettings ────────────────────────────────────────────────────────

void PpmCalibDialog::reloadFromSettings()
{
    auto& S = AppSettings::instance();
    {
        std::lock_guard<std::mutex> lk(S.ppm_calib_mutex);
        for (int c = 0; c < 6; ++c) {
            min_spins_[c]    ->setValue(S.ppm_calib[c].min_us);
            neutral_spins_[c]->setValue(S.ppm_calib[c].neutral_us);
            max_spins_[c]    ->setValue(S.ppm_calib[c].max_us);
        }
    }
    deadband_spin_->setValue(S.ppm_deadband_x1000.load() / 10.0);
}

void PpmCalibDialog::applyToSettings()
{
    auto& S = AppSettings::instance();
    {
        std::lock_guard<std::mutex> lk(S.ppm_calib_mutex);
        for (int c = 0; c < 6; ++c) {
            S.ppm_calib[c].min_us     = min_spins_[c]    ->value();
            S.ppm_calib[c].neutral_us = neutral_spins_[c]->value();
            S.ppm_calib[c].max_us     = max_spins_[c]    ->value();
        }
    }
    // Store as ×1000 (spin shows %, e.g. 5.0% → 50)
    S.ppm_deadband_x1000.store(static_cast<int>(deadband_spin_->value() * 10.0 + 0.5));
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void PpmCalibDialog::onApply()
{
    applyToSettings();
    emit calibChanged();
}

void PpmCalibDialog::onPpmReceived(QVector<int> values)
{
    for (int c = 0; c < values.size() && c < 6; ++c) {
        int us = values[c];
        live_us_[c] = us;
        val_labels_[c]->setText(QString::number(us) + " µs");
        bars_[c]->setValue(qBound(950, us, 2050));
    }
}

void PpmCalibDialog::onCaptureCenter(int ch)
{
    if (live_us_[ch] > 0)
        neutral_spins_[ch]->setValue(live_us_[ch]);
}

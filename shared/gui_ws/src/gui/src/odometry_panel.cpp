#include "gui/odometry_panel.hpp"
#include <QMouseEvent>

OdometryPanel::OdometryPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    setStyleSheet("background-color: #1e1e2e;");

    buildLayout();

    // ── Signal bridges (ROS thread → Qt main thread) ─────────────────────────
    connect(this, &OdometryPanel::telemetryUpdated,
            this, &OdometryPanel::onTelemetryUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flipperExtUpdated,
            this, &OdometryPanel::onFlipperExtUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::modeUpdated,
            this, &OdometryPanel::onModeUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flagsUpdated,
            this, &OdometryPanel::onFlagsUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::tracksUpdated,
            this, &OdometryPanel::onTracksUpdated, Qt::QueuedConnection);

    // ── ROS subscriptions ────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(10).best_effort();

    telem_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/telemetry", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 4)
                emit telemetryUpdated(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        });

    flipper_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/encoders/flipper", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 4)
                emit flipperExtUpdated(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        });

    mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/mode", qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit modeUpdated(QString::fromStdString(msg->data));
        });

    flags_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "/robot/flags", qos,
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            emit flagsUpdated(static_cast<int>(msg->data));
        });

    tracks_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/encoders/tracks", qos,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            emit tracksUpdated(msg->x, msg->y);
        });
}

// ── Layout ───────────────────────────────────────────────────────────────────

void OdometryPanel::buildLayout()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(8, 6, 8, 6);
    main_layout_->setSpacing(4);

    auto hdr_style = QString("color: #aaa; font-weight: bold; font-size: 11px;");
    auto lbl_style = QString("color: #888; font-size: 10px;");

    // ── Title ────────────────────────────────────────────────────────────────
    auto* title = new QLabel("Odometry Dashboard", this);
    title->setAlignment(Qt::AlignHCenter);
    title->setStyleSheet("color: #4fc3f7; font-weight: bold; font-size: 12px;");
    main_layout_->addWidget(title);

    main_layout_->addWidget(makeHSep());

    // ── Mode / Status row ────────────────────────────────────────────────────
    auto* status_hdr = makeHeaderLabel("Status");
    main_layout_->addWidget(status_hdr);

    auto* status_grid = new QGridLayout();
    status_grid->setSpacing(3);

    status_grid->addWidget(makeAxisLabel("Mode"), 0, 0);
    mode_label_ = makeValueLabel("--");
    status_grid->addWidget(mode_label_, 0, 1);

    status_grid->addWidget(makeAxisLabel("Uptime"), 0, 2);
    uptime_label_ = makeValueLabel("--");
    status_grid->addWidget(uptime_label_, 0, 3);

    status_grid->addWidget(makeAxisLabel("Flags"), 1, 0);
    flags_label_ = makeValueLabel("--");
    status_grid->addWidget(flags_label_, 1, 1);

    main_layout_->addLayout(status_grid);

    main_layout_->addWidget(makeHSep());

    // ── Traction ─────────────────────────────────────────────────────────────
    auto* trac_hdr = makeHeaderLabel("Traction (RPM)");
    main_layout_->addWidget(trac_hdr);

    auto* trac_row = new QHBoxLayout();
    trac_row->setSpacing(8);

    auto* left_col = new QVBoxLayout();
    left_col->setSpacing(1);
    left_col->addWidget(makeAxisLabel("Left"));
    trac_left_rpm_ = makeValueLabel("0.0");
    left_col->addWidget(trac_left_rpm_);

    auto* right_col = new QVBoxLayout();
    right_col->setSpacing(1);
    right_col->addWidget(makeAxisLabel("Right"));
    trac_right_rpm_ = makeValueLabel("0.0");
    right_col->addWidget(trac_right_rpm_);

    trac_row->addLayout(left_col);
    trac_row->addLayout(right_col);
    main_layout_->addLayout(trac_row);

    // ── Flipper separator + container ────────────────────────────────────────
    flipper_sep_ = makeHSep();
    main_layout_->addWidget(flipper_sep_);

    rebuildFlipperSection();

    main_layout_->addStretch();
}

void OdometryPanel::rebuildFlipperSection()
{
    // Remove old container if present
    if (flipper_container_) {
        main_layout_->removeWidget(flipper_container_);
        delete flipper_container_;
        flipper_container_ = nullptr;
    }

    flipper_container_ = new QWidget(this);
    auto* fl = new QVBoxLayout(flipper_container_);
    fl->setContentsMargins(0, 0, 0, 0);
    fl->setSpacing(3);

    if (robot_type_ == 0) {
        // Jaguar: single flipper set
        auto* hdr = makeHeaderLabel("Flippers (deg)");
        fl->addWidget(hdr);

        auto* row = new QHBoxLayout();
        row->addWidget(makeAxisLabel("Angle"));
        flip_angle_ = makeValueLabel("0.0");
        row->addWidget(flip_angle_, 1);
        fl->addLayout(row);

        flip_fl_ = flip_fr_ = flip_rl_ = flip_rr_ = nullptr;
    } else {
        // Dicerox: 4 independent flippers
        auto* hdr = makeHeaderLabel("Flippers (deg)");
        fl->addWidget(hdr);

        auto* grid = new QGridLayout();
        grid->setSpacing(3);

        grid->addWidget(makeAxisLabel("FL"), 0, 0);
        flip_fl_ = makeValueLabel("0.0");
        grid->addWidget(flip_fl_, 0, 1);

        grid->addWidget(makeAxisLabel("FR"), 0, 2);
        flip_fr_ = makeValueLabel("0.0");
        grid->addWidget(flip_fr_, 0, 3);

        grid->addWidget(makeAxisLabel("RL"), 1, 0);
        flip_rl_ = makeValueLabel("0.0");
        grid->addWidget(flip_rl_, 1, 1);

        grid->addWidget(makeAxisLabel("RR"), 1, 2);
        flip_rr_ = makeValueLabel("0.0");
        grid->addWidget(flip_rr_, 1, 3);

        fl->addLayout(grid);

        flip_angle_ = nullptr;
    }

    // Insert before the stretch
    int idx = main_layout_->indexOf(flipper_sep_) + 1;
    main_layout_->insertWidget(idx, flipper_container_);
}

void OdometryPanel::setRobotType(int type)
{
    if (robot_type_ == type) return;
    robot_type_ = type;
    rebuildFlipperSection();
}

// ── Factory helpers ──────────────────────────────────────────────────────────

QLabel* OdometryPanel::makeValueLabel(const QString& initial)
{
    auto* l = new QLabel(initial, this);
    l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    l->setStyleSheet("color: #4fc3f7; font-size: 12px;");
    return l;
}

QLabel* OdometryPanel::makeHeaderLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #aaa; font-weight: bold; font-size: 11px;");
    return l;
}

QLabel* OdometryPanel::makeAxisLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #888; font-size: 10px;");
    return l;
}

QFrame* OdometryPanel::makeHSep()
{
    auto* s = new QFrame(this);
    s->setFrameShape(QFrame::HLine);
    s->setStyleSheet("color: #444;");
    return s;
}

// ── Slots ────────────────────────────────────────────────────────────────────

void OdometryPanel::onTelemetryUpdated(float spd_l, float spd_r, float flip_angle, float uptime)
{
    trac_left_rpm_->setText(QString::number(spd_l, 'f', 1));
    trac_right_rpm_->setText(QString::number(spd_r, 'f', 1));
    if (robot_type_ == 0 && flip_angle_)
        flip_angle_->setText(QString::number(flip_angle, 'f', 1));
    uptime_label_->setText(QString::number(uptime, 'f', 0) + "s");
}

void OdometryPanel::onFlipperExtUpdated(float fl, float fr, float rl, float rr)
{
    if (robot_type_ == 1) {
        if (flip_fl_) flip_fl_->setText(QString::number(fl, 'f', 1));
        if (flip_fr_) flip_fr_->setText(QString::number(fr, 'f', 1));
        if (flip_rl_) flip_rl_->setText(QString::number(rl, 'f', 1));
        if (flip_rr_) flip_rr_->setText(QString::number(rr, 'f', 1));
    } else if (robot_type_ == 0 && flip_angle_) {
        // On Jaguar, flipper_sub also publishes [angle, 0, 0, 0]
        flip_angle_->setText(QString::number(fl, 'f', 1));
    }
}

void OdometryPanel::onModeUpdated(const QString& mode)
{
    mode_label_->setText(mode);

    // Color code the mode
    if (mode == "ESTOP")
        mode_label_->setStyleSheet("color: #cc3333; font-size: 12px; font-weight: bold;");
    else if (mode == "STANDBY")
        mode_label_->setStyleSheet("color: #ccaa00; font-size: 12px;");
    else if (mode == "NORMAL" || mode == "FLIPPER")
        mode_label_->setStyleSheet("color: #33cc33; font-size: 12px;");
    else
        mode_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
}

void OdometryPanel::onFlagsUpdated(int flags)
{
    QStringList parts;
    if (flags & 0x01) parts << "PPM";
    if (flags & 0x02) parts << "SENS";
    if (flags & 0x04) parts << "CAN";
    if (flags & 0x08) parts << "ESTOP";
    flags_label_->setText(parts.isEmpty() ? "none" : parts.join(" | "));
}

void OdometryPanel::onTracksUpdated(double left_rpm, double right_rpm)
{
    trac_left_rpm_->setText(QString::number(left_rpm, 'f', 1));
    trac_right_rpm_->setText(QString::number(right_rpm, 'f', 1));
}

// ── Mouse click → signal ─────────────────────────────────────────────────────
void OdometryPanel::mousePressEvent(QMouseEvent* event)
{
    Q_UNUSED(event);
    // Don't emit panelClicked — the odometry panel itself shouldn't enlarge.
    // The VideoPanel handles click-to-enlarge only for video widgets.
}

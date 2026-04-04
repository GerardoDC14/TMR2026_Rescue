#include "gui/odometry_panel.hpp"

static const char* VESC_NAMES[7] = {"?", "TL", "TR", "FL", "FR", "RL", "RR"};

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
    connect(this, &OdometryPanel::vescStatusUpdated,
            this, &OdometryPanel::onVescStatusUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::mainMotorUpdated,
            this, &OdometryPanel::onMainMotorUpdated, Qt::QueuedConnection);

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

    // VESC status (secondary robot): [vesc_id, erpm, current_A, duty, temp_fet, temp_motor, voltage]
    vesc_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/vesc_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7)
                emit vescStatusUpdated(
                    static_cast<int>(msg->data[0]),
                    msg->data[1], msg->data[2], msg->data[3],
                    msg->data[4], msg->data[5], msg->data[6]);
        });

    // Main robot motor commands: [left_duty, right_duty, flipper_duty] (−1..+1)
    main_motor_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/main_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 3)
                emit mainMotorUpdated(msg->data[0], msg->data[1], msg->data[2]);
        });
}

// ── Layout ───────────────────────────────────────────────────────────────────

void OdometryPanel::buildLayout()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(8, 6, 8, 6);
    main_layout_->setSpacing(4);

    // ── Mode / Flags row (uptime moved to dashboard panel) ───────────────────
    auto* status_hdr = makeHeaderLabel("Telemetry Status");
    main_layout_->addWidget(status_hdr);

    auto* status_grid = new QGridLayout();
    status_grid->setSpacing(3);

    status_grid->addWidget(makeAxisLabel("Mode"), 0, 0);
    mode_label_ = makeValueLabel("--");
    status_grid->addWidget(mode_label_, 0, 1);

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
    trac_left_rpm_ = makeValueLabel("--");
    left_col->addWidget(trac_left_rpm_);

    auto* right_col = new QVBoxLayout();
    right_col->setSpacing(1);
    right_col->addWidget(makeAxisLabel("Right"));
    trac_right_rpm_ = makeValueLabel("--");
    right_col->addWidget(trac_right_rpm_);

    trac_row->addLayout(left_col);
    trac_row->addLayout(right_col);
    main_layout_->addLayout(trac_row);

    // ── Flipper separator + container ────────────────────────────────────────
    flipper_sep_ = makeHSep();
    main_layout_->addWidget(flipper_sep_);

    rebuildFlipperSection();

    // ── VESC telemetry section (secondary robot) ─────────────────────────────
    main_layout_->addWidget(makeHSep());

    vesc_section_ = new QWidget(this);
    {
        auto* vl = new QVBoxLayout(vesc_section_);
        vl->setContentsMargins(0, 0, 0, 0);
        vl->setSpacing(3);

        vl->addWidget(makeHeaderLabel("Motor Telemetry"));

        // Column headers
        auto* hdr_grid = new QGridLayout();
        hdr_grid->setSpacing(2);
        hdr_grid->addWidget(makeAxisLabel("ID"),    0, 0);
        hdr_grid->addWidget(makeAxisLabel("eRPM"),  0, 1);
        hdr_grid->addWidget(makeAxisLabel("A"),     0, 2);
        hdr_grid->addWidget(makeAxisLabel("Duty"),  0, 3);
        hdr_grid->addWidget(makeAxisLabel("Tfet"),  0, 4);
        hdr_grid->addWidget(makeAxisLabel("Tmot"),  0, 5);
        vl->addLayout(hdr_grid);

        // One row per VESC (IDs 1-6)
        for (int id = 1; id <= 6; ++id) {
            auto* row = new QGridLayout();
            row->setSpacing(2);

            auto* id_lbl = makeAxisLabel(VESC_NAMES[id]);
            row->addWidget(id_lbl, 0, 0);

            vesc_rows_[id].erpm      = makeValueLabel("--");
            vesc_rows_[id].current   = makeValueLabel("--");
            vesc_rows_[id].duty      = makeValueLabel("--");
            vesc_rows_[id].temp_fet  = makeValueLabel("--");
            vesc_rows_[id].temp_motor = makeValueLabel("--");

            // Smaller font for the dense table
            for (QLabel* l : {vesc_rows_[id].erpm, vesc_rows_[id].current,
                               vesc_rows_[id].duty, vesc_rows_[id].temp_fet,
                               vesc_rows_[id].temp_motor}) {
                l->setStyleSheet("color: #4fc3f7; font-size: 10px;");
            }

            row->addWidget(vesc_rows_[id].erpm,      0, 1);
            row->addWidget(vesc_rows_[id].current,   0, 2);
            row->addWidget(vesc_rows_[id].duty,      0, 3);
            row->addWidget(vesc_rows_[id].temp_fet,  0, 4);
            row->addWidget(vesc_rows_[id].temp_motor, 0, 5);
            vl->addLayout(row);
        }
    }
    main_layout_->addWidget(vesc_section_);

    // ── Main robot motor commands section (primary robot) ────────────────────
    main_layout_->addWidget(makeHSep());

    main_motor_section_ = new QWidget(this);
    {
        auto* vl = new QVBoxLayout(main_motor_section_);
        vl->setContentsMargins(0, 0, 0, 0);
        vl->setSpacing(3);

        vl->addWidget(makeHeaderLabel("Motor Commands"));

        auto* mg = new QGridLayout();
        mg->setSpacing(3);
        mg->addWidget(makeAxisLabel("Left"),    0, 0);
        mg->addWidget(makeAxisLabel("Right"),   0, 1);
        mg->addWidget(makeAxisLabel("Flipper"), 0, 2);
        main_left_duty_  = makeValueLabel("--");
        main_right_duty_ = makeValueLabel("--");
        main_flip_duty_  = makeValueLabel("--");
        mg->addWidget(main_left_duty_,  1, 0);
        mg->addWidget(main_right_duty_, 1, 1);
        mg->addWidget(main_flip_duty_,  1, 2);
        vl->addLayout(mg);
    }
    main_layout_->addWidget(main_motor_section_);

    main_layout_->addStretch();

    // Show/hide motor sections based on initial robot type
    updateMotorSectionVisibility();
}

void OdometryPanel::rebuildFlipperSection()
{
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
        auto* hdr = makeHeaderLabel("Flippers");
        fl->addWidget(hdr);

        
        auto* row = new QHBoxLayout();
        flip_angle_ = makeValueLabel("--");
        row->addWidget(flip_angle_, 1);
        fl->addLayout(row);

        flip_fl_ = flip_fr_ = flip_rl_ = flip_rr_ = nullptr;
    } else {
        auto* hdr = makeHeaderLabel("Flippers");
        fl->addWidget(hdr);

        auto* grid = new QGridLayout();
        grid->setSpacing(3);

        grid->addWidget(makeAxisLabel("FL"), 0, 0);
        flip_fl_ = makeValueLabel("--");
        grid->addWidget(flip_fl_, 0, 1);

        grid->addWidget(makeAxisLabel("FR"), 0, 2);
        flip_fr_ = makeValueLabel("--");
        grid->addWidget(flip_fr_, 0, 3);

        grid->addWidget(makeAxisLabel("RL"), 1, 0);
        flip_rl_ = makeValueLabel("--");
        grid->addWidget(flip_rl_, 1, 1);

        grid->addWidget(makeAxisLabel("RR"), 1, 2);
        flip_rr_ = makeValueLabel("--");
        grid->addWidget(flip_rr_, 1, 3);

        fl->addLayout(grid);

        flip_angle_ = nullptr;
    }

    int idx = main_layout_->indexOf(flipper_sep_) + 1;
    main_layout_->insertWidget(idx, flipper_container_);
}

void OdometryPanel::updateMotorSectionVisibility()
{
    if (vesc_section_)       vesc_section_->setVisible(robot_type_ == 1);
    if (main_motor_section_) main_motor_section_->setVisible(robot_type_ == 0);
}

void OdometryPanel::setRobotType(int type)
{
    if (robot_type_ == type) return;
    robot_type_ = type;
    rebuildFlipperSection();
    updateMotorSectionVisibility();
}

// ── Factory helpers ──────────────────────────────────────────────────────────

QLabel* OdometryPanel::makeValueLabel(const QString& initial)
{
    auto* l = new QLabel(initial, this);
    l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    l->setStyleSheet("color: #4fc3f7; font-size: 13px;");
    return l;
}

QLabel* OdometryPanel::makeHeaderLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #aaa; font-weight: bold;");
    return l;
}

QLabel* OdometryPanel::makeAxisLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #888;");
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

void OdometryPanel::onTelemetryUpdated(float spd_l, float spd_r, float flip_angle, float /*uptime*/)
{
    trac_left_rpm_->setText(QString::number(spd_l, 'f', 1));
    trac_right_rpm_->setText(QString::number(spd_r, 'f', 1));
    if (robot_type_ == 0 && flip_angle_)
        flip_angle_->setText(QString::number(flip_angle, 'f', 1) + "°");
}

void OdometryPanel::onFlipperExtUpdated(float fl, float fr, float rl, float rr)
{
    if (robot_type_ == 1) {
        if (flip_fl_) flip_fl_->setText(QString::number(fl, 'f', 1) + "°");
        if (flip_fr_) flip_fr_->setText(QString::number(fr, 'f', 1) + "°");
        if (flip_rl_) flip_rl_->setText(QString::number(rl, 'f', 1) + "°");
        if (flip_rr_) flip_rr_->setText(QString::number(rr, 'f', 1) + "°");
    } else if (robot_type_ == 0 && flip_angle_) {
        flip_angle_->setText(QString::number(fl, 'f', 1));
    }
}

void OdometryPanel::onModeUpdated(const QString& mode)
{
    mode_label_->setText(mode);

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

void OdometryPanel::onVescStatusUpdated(int id, float erpm, float current, float duty,
                                        float temp_fet, float temp_motor, float /*voltage*/)
{
    if (id < 1 || id > 6) return;
    auto& row = vesc_rows_[id];
    row.erpm->setText(QString::number(static_cast<int>(erpm)));
    row.current->setText(QString::number(current, 'f', 1) + "A");
    row.duty->setText(QString::number(duty * 100.0f, 'f', 0) + "%");
    row.temp_fet->setText(QString::number(temp_fet, 'f', 0) + "°");
    row.temp_motor->setText(QString::number(temp_motor, 'f', 0) + "°");
}

void OdometryPanel::onMainMotorUpdated(float left_duty, float right_duty, float flipper_duty)
{
    auto fmt = [](float v) {
        return QString::number(v * 100.0f, 'f', 0) + "%";
    };
    if (main_left_duty_)  main_left_duty_->setText(fmt(left_duty));
    if (main_right_duty_) main_right_duty_->setText(fmt(right_duty));
    if (main_flip_duty_)  main_flip_duty_->setText(fmt(flipper_duty));
}


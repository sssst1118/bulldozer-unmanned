/**
 * @file tab9_diag_snapshot.inl
 * @brief Tab9 诊断录制 — 点击开始录制N秒全链路数据
 * @author dozer-dev
 * @date 2026-04-14
 *
 * 功能:
 *   - 点"开始录制"按钮, 实时采集N秒全链路数据, 写入CSV
 *   - 录制中显示进度条和剩余时间, 可随时取消
 *   - 可自定义: 保存路径、录制时长、问题描述
 *   - 操作事件持续记录 (按钮点击/模式切换等)
 *
 * 记录字段 (每行, 10Hz):
 *   时间戳, ENU_X, ENU_Y, 航向, 执行器状态, 路径点索引,
 *   目标距离, 航向偏差, walk_state, decision_status,
 *   V_right, V_left, terminal_flag, 铲刀高度左, 铲刀高度右,
 *   车速, 发动机转速, 挡位
 *
 * 操作事件 (触发时追加):
 *   时间戳, 事件类型, 描述
 */

// ---- 数据结构定义在 manual_control_widget.h 中 (DiagSample, DiagEvent) ----

// ---- Tab9 UI 创建 ----

void ManualControlWidget::setupTab9_DiagSnapshot(QTabWidget* tabs)
{
    auto* tab = new QWidget();
    auto* layout = new QVBoxLayout(tab);
    layout->setSpacing(10);
    layout->setContentsMargins(12, 12, 12, 12);

    //=== 录制设置 ===
    auto* settingsG = makeGroup("录制设置");
    auto* sL = new QGridLayout(settingsG);

    sL->addWidget(new QLabel("保存路径:"), 0, 0);
    edit_diag_path_ = new QLineEdit(QDir::homePath() + "/dozer_diag");
    edit_diag_path_->setFont(MONO_FONT);
    sL->addWidget(edit_diag_path_, 0, 1, 1, 3);
    auto* btn_browse = new IOSButton("浏览...", IOSButton::Secondary);
    btn_browse->setFixedWidth(80);
    connect(btn_browse, &QPushButton::clicked, [this]{
        QString dir = QFileDialog::getExistingDirectory(this, "选择保存路径", edit_diag_path_->text());
        if (!dir.isEmpty()) edit_diag_path_->setText(dir);
    });
    sL->addWidget(btn_browse, 0, 4);

    sL->addWidget(new QLabel("录制时长(秒):"), 1, 0);
    spin_diag_duration_ = makeSpin(1, 300, 10, 1, 0);
    spin_diag_duration_->setFixedWidth(100);
    sL->addWidget(spin_diag_duration_, 1, 1);

    sL->addWidget(new QLabel("问题描述:"), 2, 0);
    edit_diag_desc_ = new QLineEdit();
    edit_diag_desc_->setPlaceholderText("简要描述出了什么问题...");
    edit_diag_desc_->setFont(QFont("", 11));
    sL->addWidget(edit_diag_desc_, 2, 1, 1, 4);

    layout->addWidget(settingsG);

    //=== 操作按钮 ===
    auto* btnRow = new QHBoxLayout();
    // Tab9 开始录制 — Warn (橙色) 语义
    btn_diag_save_ = new IOSButton("开始录制", IOSButton::Warn);
    btn_diag_save_->setFixedHeight(50);
    btn_diag_save_->setFont(QFont("", 14, QFont::Bold));
    btnRow->addWidget(btn_diag_save_);

    // Tab9 取消录制 — Danger (红色) 语义
    btn_diag_cancel_ = new IOSButton("取消", IOSButton::Danger);
    btn_diag_cancel_->setFixedHeight(50);
    btn_diag_cancel_->setFont(QFont("", 12, QFont::Bold));
    btn_diag_cancel_->setFixedWidth(120);
    btn_diag_cancel_->setVisible(false);
    btnRow->addWidget(btn_diag_cancel_);
    layout->addLayout(btnRow);

    //=== 录制进度 ===
    auto* progressG = makeGroup("录制状态");
    auto* pgL = new QVBoxLayout(progressG);
    diag_progress_ = new QProgressBar();
    diag_progress_->setMinimum(0);
    diag_progress_->setMaximum(100);
    diag_progress_->setValue(0);
    diag_progress_->setFixedHeight(30);
    diag_progress_->setFormat("%p%");
    diag_progress_->setStyleSheet(
        "QProgressBar { border:1px solid #D1D1D6; border-radius:8px; background:#F9FAFB; text-align:center; font-weight:bold; }"
        "QProgressBar::chunk { background:#FF9500; border-radius:7px; }");
    pgL->addWidget(diag_progress_);

    label_diag_status_ = new QLabel("就绪");
    label_diag_status_->setFont(MONO_FONT);
    label_diag_status_->setAlignment(Qt::AlignCenter);
    pgL->addWidget(label_diag_status_);

    label_diag_last_save_ = new QLabel("");
    label_diag_last_save_->setFont(MONO_FONT);
    label_diag_last_save_->setAlignment(Qt::AlignCenter);
    pgL->addWidget(label_diag_last_save_);
    layout->addWidget(progressG);

    //=== 操作事件日志 (实时显示) ===
    auto* eventG = makeGroup("操作事件日志");
    auto* eL = new QVBoxLayout(eventG);
    text_diag_events_ = new QTextEdit();
    text_diag_events_->setReadOnly(true);
    text_diag_events_->setFont(MONO_FONT);
    text_diag_events_->setMaximumHeight(200);
    eL->addWidget(text_diag_events_);
    layout->addWidget(eventG);

    layout->addStretch();

    //=== 初始化 ===
    diag_sample_rate_ = 10;  // 10Hz采样
    diag_recording_ = false;

    //=== 采样定时器 (10Hz, 仅录制时工作) ===
    diag_timer_ = new QTimer(this);
    connect(diag_timer_, &QTimer::timeout, [this]{
        // 采样一帧
        DiagSample s;
        s.timestamp       = ros::Time::now().toSec();
        s.enu_x           = diag_enu_x_;
        s.enu_y           = diag_enu_y_;
        s.heading         = heading_deg_;
        s.exec_state      = diag_exec_state_;
        s.waypoint_index  = diag_wp_index_;
        s.target_dist     = diag_target_dist_;
        s.heading_error   = diag_heading_error_;
        s.walk_state      = walk_state_val_;
        s.decision_status = decision_status_val_;
        s.v_right         = v_right_val_;
        s.v_left          = v_left_val_;
        s.terminal_flag   = terminal_flag_val_;
        s.mold_height_l   = mold_left_mm_;
        s.mold_height_r   = mold_right_mm_;
        s.vehicle_speed   = vehicle_speed_;
        s.engine_rpm      = engine_rpm_;
        s.gear_pos        = gear_pos_;
        s.gear_dir        = gear_dir_;
        diag_samples_.push_back(s);

        // 更新进度
        diag_recorded_count_++;
        int total = diag_target_samples_;
        int pct = std::min(100, diag_recorded_count_ * 100 / total);
        diag_progress_->setValue(pct);
        double elapsed = (double)diag_recorded_count_ / diag_sample_rate_;
        double remain = (double)(total - diag_recorded_count_) / diag_sample_rate_;
        label_diag_status_->setText(QString("录制中... %1/%2 秒 (剩余 %3 秒)")
            .arg(elapsed, 0, 'f', 1)
            .arg((double)total / diag_sample_rate_, 0, 'f', 0)
            .arg(remain, 0, 'f', 1));

        // 到达目标, 停止录制并保存
        if (diag_recorded_count_ >= total) {
            diagStopAndSave();
        }
    });

    //=== 开始录制 ===
    connect(btn_diag_save_, &QPushButton::clicked, [this]{
        if (diag_recording_) return;

        int duration = static_cast<int>(spin_diag_duration_->value());
        diag_target_samples_ = duration * diag_sample_rate_;
        diag_recorded_count_ = 0;
        diag_samples_.clear();
        diag_recording_ = true;

        // 记录录制开始时间戳 (用于筛选事件)
        diag_record_start_time_ = ros::Time::now().toSec();

        // UI切换
        btn_diag_save_->setVisible(false);
        btn_diag_cancel_->setVisible(true);
        edit_diag_path_->setEnabled(false);
        spin_diag_duration_->setEnabled(false);
        diag_progress_->setValue(0);
        label_diag_status_->setText("录制中... 0/0 秒");
        label_diag_status_->setStyleSheet("color:#FF9500;font-weight:bold;");

        recordDiagEvent("REC", "开始录制 " + QString::number(duration) + "秒");

        diag_timer_->start(1000 / diag_sample_rate_);
    });

    //=== 取消录制 ===
    connect(btn_diag_cancel_, &QPushButton::clicked, [this]{
        if (!diag_recording_) return;

        diag_timer_->stop();
        diag_recording_ = false;
        diag_samples_.clear();

        // UI恢复
        btn_diag_save_->setVisible(true);
        btn_diag_cancel_->setVisible(false);
        edit_diag_path_->setEnabled(true);
        spin_diag_duration_->setEnabled(true);
        diag_progress_->setValue(0);
        label_diag_status_->setText("已取消");
        label_diag_status_->setStyleSheet("color:#AEAEB2;");

        recordDiagEvent("REC", "取消录制");
    });

    //=== 诊断数据订阅 ===
    sub_diag_exec_state_ = nh_.subscribe<std_msgs::Float64>("/decision/exec_state", 1,
        [this](const std_msgs::Float64::ConstPtr& msg){ diag_exec_state_ = static_cast<int>(msg->data); });
    sub_diag_wp_index_ = nh_.subscribe<std_msgs::Float64>("/decision/waypoint_index", 1,
        [this](const std_msgs::Float64::ConstPtr& msg){ diag_wp_index_ = static_cast<int>(msg->data); });
    sub_diag_error_ = nh_.subscribe<geometry_msgs::Point>("/control/error", 1,
        [this](const geometry_msgs::Point::ConstPtr& msg){
            diag_target_dist_ = msg->x;
            diag_heading_error_ = msg->y;
        });
    sub_diag_enu_ = nh_.subscribe<geometry_msgs::Point>("/control/enu_position", 1,
        [this](const geometry_msgs::Point::ConstPtr& msg){
            diag_enu_x_ = msg->x;
            diag_enu_y_ = msg->y;
        });

    tabs->addTab(tab, "\U0001F4F8 \u8BCA\u65AD\u5FEB\u7167");
}

// ---- 录制完成, 写入CSV ----

void ManualControlWidget::diagStopAndSave()
{
    diag_timer_->stop();
    diag_recording_ = false;

    QString desc = edit_diag_desc_->text().trimmed();
    QString dir = edit_diag_path_->text();
    QDir().mkpath(dir);

    QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString filepath = dir + "/diag_" + ts + ".csv";

    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        label_diag_status_->setText("保存失败: " + file.errorString());
        label_diag_status_->setStyleSheet("color:#DC2626;");
        btn_diag_save_->setVisible(true);
        btn_diag_cancel_->setVisible(false);
        edit_diag_path_->setEnabled(true);
        spin_diag_duration_->setEnabled(true);
        return;
    }

    QTextStream out(&file);

    // 文件头
    out << "# 诊断录制\n";
    out << "# 时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") << "\n";
    out << "# 时长: " << static_cast<int>(spin_diag_duration_->value()) << " 秒\n";
    out << "# 采样: " << diag_samples_.size() << " 条 @ " << diag_sample_rate_ << "Hz\n";
    out << "# 描述: " << (desc.isEmpty() ? "(无)" : desc) << "\n";
    out << "#\n";

    // 操作事件 (录制期间的)
    out << "# ===== 操作事件 =====\n";
    out << "# event_time,event_type,event_desc\n";
    for (const auto& e : diag_events_) {
        if (e.timestamp >= diag_record_start_time_) {
            out << "# EVENT," << QString::number(e.timestamp, 'f', 3) << ","
                << e.type << "," << e.description << "\n";
        }
    }
    out << "#\n";

    // 数据表头
    out << "timestamp,enu_x,enu_y,heading,exec_state,wp_index,"
        << "target_dist,heading_error,walk_state,decision_status,"
        << "v_right,v_left,terminal_flag,mold_h_left,mold_h_right,"
        << "vehicle_speed,engine_rpm,gear_pos,gear_dir\n";

    // 数据
    for (const auto& s : diag_samples_) {
        out << QString::number(s.timestamp, 'f', 3) << ","
            << QString::number(s.enu_x, 'f', 3) << ","
            << QString::number(s.enu_y, 'f', 3) << ","
            << QString::number(s.heading, 'f', 2) << ","
            << s.exec_state << "," << s.waypoint_index << ","
            << QString::number(s.target_dist, 'f', 3) << ","
            << QString::number(s.heading_error, 'f', 2) << ","
            << QString::number(s.walk_state, 'f', 0) << ","
            << QString::number(s.decision_status, 'f', 0) << ","
            << QString::number(s.v_right, 'f', 1) << ","
            << QString::number(s.v_left, 'f', 1) << ","
            << QString::number(s.terminal_flag, 'f', 0) << ","
            << QString::number(s.mold_height_l, 'f', 0) << ","
            << QString::number(s.mold_height_r, 'f', 0) << ","
            << QString::number(s.vehicle_speed, 'f', 3) << ","
            << QString::number(s.engine_rpm, 'f', 0) << ","
            << s.gear_pos << "," << s.gear_dir << "\n";
    }

    file.close();
    diag_samples_.clear();

    // UI恢复
    btn_diag_save_->setVisible(true);
    btn_diag_cancel_->setVisible(false);
    edit_diag_path_->setEnabled(true);
    spin_diag_duration_->setEnabled(true);
    diag_progress_->setValue(100);
    label_diag_status_->setText("录制完成");
    label_diag_status_->setStyleSheet("color:#34C759;font-weight:bold;");
    label_diag_last_save_->setText("已保存: " + filepath);
    label_diag_last_save_->setStyleSheet("color:#34C759;");
    edit_diag_desc_->clear();

    recordDiagEvent("REC", "录制完成 → " + filepath);
}

// ---- 操作事件记录接口 ----

void ManualControlWidget::recordDiagEvent(const QString& type, const QString& desc)
{
    DiagEvent e;
    e.timestamp = ros::Time::now().toSec();
    e.type = type;
    e.description = desc;
    diag_events_.push_back(e);

    // 事件日志最多保留500条
    if (diag_events_.size() > 500)
        diag_events_.pop_front();

    // 显示到界面
    if (text_diag_events_) {
        QString line = QString("[%1] %2: %3")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
            .arg(type).arg(desc);
        text_diag_events_->append(line);
        QTextCursor c = text_diag_events_->textCursor();
        c.movePosition(QTextCursor::End);
        text_diag_events_->setTextCursor(c);
    }
}

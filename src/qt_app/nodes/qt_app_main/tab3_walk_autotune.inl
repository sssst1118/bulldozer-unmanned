/**
 * @file tab3_walk_autotune.inl
 * @brief 行走PID自动整定 — 阶跃响应法, 迭代收敛
 * @author dozer-dev
 * @date 2026-04-14
 *
 * 功能:
 *   - 对距离(X)或航向(θ)通路执行正/反阶跃, 记录响应曲线
 *   - 分析上升时间、超调量、稳态误差、震荡
 *   - 根据分析结果自动调整PID参数
 *   - 支持手动逐轮确认和全自动迭代两种模式
 *
 * 集成到 Tab3 行走PID分组下方
 */

// 复用铲刀整定的 AutoTuneCurveWidget (已在 tab3_autotune.inl 中定义)

// ============================================================================
// Tab3 行走自动整定UI
// ============================================================================
void ManualControlWidget::setupTab3_WalkAutoTune(QVBoxLayout* layout)
{
    auto* wtG = makeGroup("行走 PID 自动整定");
    auto* wtL = new QVBoxLayout(wtG);

    //=== 参数行 ===
    auto* paramRow = new QHBoxLayout();

    paramRow->addWidget(new QLabel("通道:"));
    // iOS SegmentedControl 2 段: 距离(X) / 航向(θ) — 行走自动整定通道
    combo_wt_channel_ = new SegmentedControl({"距离(X)", "航向(θ)"});
    combo_wt_channel_->setFixedWidth(160);
    paramRow->addWidget(combo_wt_channel_);

    paramRow->addWidget(new QLabel("阶跃幅度:"));
    spin_wt_step_ = makeSpin(0.1, 50, 2.0, 0.1, 1);
    spin_wt_step_->setFixedWidth(80);
    spin_wt_step_->setToolTip("距离通道: 米, 航向通道: 度");
    paramRow->addWidget(spin_wt_step_);
    // 单位标签跟随通道切换
    auto* label_wt_unit = new QLabel("m");
    paramRow->addWidget(label_wt_unit);
    connect(combo_wt_channel_, &SegmentedControl::selectionChanged, [=](int idx){
        if (idx == 0) { label_wt_unit->setText("m"); spin_wt_step_->setValue(2.0); }
        else          { label_wt_unit->setText("°"); spin_wt_step_->setValue(15.0); }
    });

    paramRow->addWidget(new QLabel("稳定时间(秒):"));
    spin_wt_settle_sec_ = makeSpin(1, 30, 3, 0.5, 1);
    spin_wt_settle_sec_->setFixedWidth(80);
    paramRow->addWidget(spin_wt_settle_sec_);

    paramRow->addWidget(new QLabel("最大轮次:"));
    spin_wt_max_iter_ = new QSpinBox();
    spin_wt_max_iter_->setRange(1, 50);
    spin_wt_max_iter_->setValue(10);
    spin_wt_max_iter_->setFixedWidth(60);
    paramRow->addWidget(spin_wt_max_iter_);

    paramRow->addStretch();
    wtL->addLayout(paramRow);

    //=== 按钮行 ===
    auto* btnRow = new QHBoxLayout();
    // IOSButton::Primary — 开始标定 (iOS 蓝)
    btn_wt_start_ = new IOSButton("开始标定", IOSButton::Primary);
    btn_wt_start_->setFixedHeight(40);
    btnRow->addWidget(btn_wt_start_);

    // IOSButton::Warn — 应用并继续 (iOS 橙)
    btn_wt_apply_ = new IOSButton("应用并继续", IOSButton::Warn);
    btn_wt_apply_->setFixedHeight(40);
    btn_wt_apply_->setEnabled(false);
    btnRow->addWidget(btn_wt_apply_);

    // IOSButton::Success — 自动继续 (iOS 绿)
    btn_wt_auto_ = new IOSButton("自动继续", IOSButton::Success);
    btn_wt_auto_->setFixedHeight(40);
    btn_wt_auto_->setEnabled(false);
    btnRow->addWidget(btn_wt_auto_);

    // IOSButton::Danger — 停止 (iOS 红)
    btn_wt_stop_ = new IOSButton("停止", IOSButton::Danger);
    btn_wt_stop_->setFixedHeight(40);
    btn_wt_stop_->setEnabled(false);
    btnRow->addWidget(btn_wt_stop_);

    wtL->addLayout(btnRow);

    //=== 状态行 ===
    auto* statusRow = new QHBoxLayout();
    label_wt_status_ = new QLabel("就绪");
    label_wt_status_->setFont(QFont("", 12, QFont::Bold));
    label_wt_status_->setStyleSheet("color:#AEAEB2;");
    statusRow->addWidget(label_wt_status_);
    statusRow->addStretch();
    label_wt_iter_ = new QLabel("");
    label_wt_iter_->setFont(MONO_FONT);
    statusRow->addWidget(label_wt_iter_);
    wtL->addLayout(statusRow);

    //=== 响应曲线 ===
    auto* curveW = new AutoTuneCurveWidget(this);
    wt_curve_widget_ = curveW;
    wtL->addWidget(wt_curve_widget_);

    //=== 分析结果 ===
    label_wt_analysis_ = new QLabel("");
    label_wt_analysis_->setFont(MONO_FONT);
    label_wt_analysis_->setWordWrap(true);
    label_wt_analysis_->setStyleSheet("background:#F9FAFB;border-radius:8px;padding:8px;");
    wtL->addWidget(label_wt_analysis_);

    //=== 建议参数 ===
    label_wt_suggest_ = new QLabel("");
    label_wt_suggest_->setFont(QFont("", 11, QFont::Bold));
    label_wt_suggest_->setWordWrap(true);
    label_wt_suggest_->setStyleSheet("color:#007AFF;padding:4px;");
    wtL->addWidget(label_wt_suggest_);

    layout->addWidget(wtG);

    //=== 采样定时器 (50Hz) ===
    wt_timer_ = new QTimer(this);
    connect(wt_timer_, &QTimer::timeout, this, &ManualControlWidget::wtSampleTick);

    //=== 按钮连接 ===
    connect(btn_wt_start_, &QPushButton::clicked, [this]{
        if (wt_phase_ != AT_IDLE) return;
        wt_channel_ = (combo_wt_channel_->selectedIndex() == 0) ? WT_CH_DISTANCE : WT_CH_HEADING;
        wt_iteration_ = 0;
        wt_auto_mode_ = false;

        btn_wt_start_->setEnabled(false);
        btn_wt_stop_->setEnabled(true);
        btn_wt_apply_->setEnabled(false);
        btn_wt_auto_->setEnabled(false);
        combo_wt_channel_->setEnabled(false);

        wt_iteration_++;
        label_wt_iter_->setText(QString("第 %1 轮").arg(wt_iteration_));
        wtStartStep(true);
    });

    connect(btn_wt_apply_, &QPushButton::clicked, [this]{
        if (wt_phase_ != AT_WAIT_USER) return;
        wtApplyNewParams();
        if (wt_iteration_ >= spin_wt_max_iter_->value()) {
            label_wt_status_->setText("达到最大轮次,整定结束");
            label_wt_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            wtStop();
            return;
        }
        wt_iteration_++;
        label_wt_iter_->setText(QString("第 %1 轮").arg(wt_iteration_));
        wtStartStep(true);
    });

    connect(btn_wt_auto_, &QPushButton::clicked, [this]{
        if (wt_phase_ != AT_WAIT_USER) return;
        wt_auto_mode_ = true;
        btn_wt_auto_->setEnabled(false);
        btn_wt_apply_->setEnabled(false);
        wtApplyNewParams();
        if (wt_iteration_ >= spin_wt_max_iter_->value()) {
            label_wt_status_->setText("达到最大轮次,整定结束");
            label_wt_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            wtStop();
            return;
        }
        wt_iteration_++;
        label_wt_iter_->setText(QString("第 %1 轮").arg(wt_iteration_));
        wtStartStep(true);
    });

    connect(btn_wt_stop_, &QPushButton::clicked, [this]{ wtStop(); });
}

// ============================================================================
// 开始一次阶跃
// ============================================================================
void ManualControlWidget::wtStartStep(bool forward)
{
    double step = spin_wt_step_->value();

    if (forward) {
        wt_phase_ = AT_STEP_UP;
        wt_samples_fwd_.clear();
        wt_result_fwd_ = {};
        wt_samples_rev_.clear();
        wt_result_rev_ = {};
        label_wt_status_->setText("正向阶跃中...");
        label_wt_status_->setStyleSheet("color:#007AFF;font-weight:bold;");
    } else {
        wt_phase_ = AT_STEP_DN;
        wt_samples_rev_.clear();
        wt_result_rev_ = {};
        label_wt_status_->setText("反向阶跃中...");
        label_wt_status_->setStyleSheet("color:#34C759;font-weight:bold;");
    }

    // 打开主开关
    {
        std_msgs::Float64 f;
        f.data = 1.0;
        pub_main_switch_.publish(f);
        pub_main_switch_ctrl_.publish(f);
    }

    if (wt_channel_ == WT_CH_DISTANCE) {
        // 距离通道: walk_state=2(直行), terminal=step_m
        std_msgs::Float64 ws;
        ws.data = 2; // 直行模式
        pub_walk_state_.publish(ws);

        std_msgs::Float64MultiArray t;
        t.data.resize(3, 0);
        t.data[0] = forward ? step : -step; // X方向距离(m)
        pub_terminal_.publish(t);

        std_msgs::Float64MultiArray r;
        r.data.resize(2, 0);
        r.data[0] = 0.5; // 参考速度(m/s), 保守值
        pub_reference_.publish(r);

        wt_baseline_ = 0; // ENU距离从0开始
        wt_target_val_ = forward ? step : -step;
    } else {
        // 航向通道: walk_state=1(旋转), terminal=step_deg
        std_msgs::Float64 ws;
        ws.data = 1; // 旋转模式
        pub_walk_state_.publish(ws);

        std_msgs::Float64MultiArray t;
        t.data.resize(3, 0);
        t.data[2] = forward ? step : -step; // 旋转角度(deg)
        pub_terminal_.publish(t);

        std_msgs::Float64MultiArray r;
        r.data.resize(2, 0);
        r.data[1] = 5.0; // 参考角速度(deg/s)
        pub_reference_.publish(r);

        wt_baseline_ = heading_deg_;
        wt_target_val_ = heading_deg_ + (forward ? step : -step);
    }

    wt_step_start_time_ = ros::Time::now().toSec();
    wt_settle_deadline_ = 0;

    wt_timer_->start(20); // 50Hz

    recordDiagEvent("WT", QString("行走阶跃 %1 → %2 (通道:%3 轮次:%4)")
        .arg(forward ? "正向" : "反向")
        .arg(wt_target_val_, 0, 'f', 2)
        .arg(wt_channel_ == WT_CH_DISTANCE ? "距离" : "航向")
        .arg(wt_iteration_));
}

// ============================================================================
// 采样定时器回调 (50Hz)
// ============================================================================
void ManualControlWidget::wtSampleTick()
{
    double now = ros::Time::now().toSec();
    double t = now - wt_step_start_time_;

    AutoTuneSample s;
    s.time_sec = t;

    if (wt_channel_ == WT_CH_DISTANCE) {
        s.target = wt_target_val_;
        // 用control node的error来反推已走距离
        // error_x = target_dist - actual_dist, 所以 actual = target - error
        s.actual = wt_target_val_ - diag_target_dist_;
        s.control_out = (v_right_val_ + v_left_val_) / 2.0;
    } else {
        s.target = wt_target_val_;
        s.actual = heading_deg_;
        s.control_out = (v_right_val_ - v_left_val_) / 2.0;
    }

    bool is_fwd = (wt_phase_ == AT_STEP_UP || wt_phase_ == AT_WAIT_UP);
    auto& samples = is_fwd ? wt_samples_fwd_ : wt_samples_rev_;
    samples.push_back(s);

    // 更新曲线
    auto* cw = static_cast<AutoTuneCurveWidget*>(wt_curve_widget_);
    cw->samples_up = &wt_samples_fwd_;
    cw->samples_dn = &wt_samples_rev_;
    wt_curve_widget_->update();

    // 判断到位: terminal_flag 为 1
    double settle_sec = spin_wt_settle_sec_->value();
    bool reached = (terminal_flag_val_ > 0.5);

    if (reached) {
        if (wt_settle_deadline_ == 0) {
            wt_settle_deadline_ = now + settle_sec;
            if (is_fwd) wt_phase_ = AT_WAIT_UP;
            else        wt_phase_ = AT_WAIT_DN;
            label_wt_status_->setText(is_fwd ? "正向 等待稳定..." : "反向 等待稳定...");
        }
        if (now >= wt_settle_deadline_) {
            wt_timer_->stop();
            // 停车
            std_msgs::Float64 ws; ws.data = 0;
            pub_walk_state_.publish(ws);

            if (is_fwd) {
                // 等一会儿再反向
                QTimer::singleShot(1000, this, [this]{ wtStartStep(false); });
            } else {
                wt_phase_ = AT_ANALYZE;
                wtShowResults();
            }
            return;
        }
    } else {
        wt_settle_deadline_ = 0;
        if (is_fwd) wt_phase_ = AT_STEP_UP;
        else        wt_phase_ = AT_STEP_DN;
    }

    // 超时保护: 30秒
    if (t > 30.0) {
        wt_timer_->stop();
        std_msgs::Float64 ws; ws.data = 0;
        pub_walk_state_.publish(ws);

        if (is_fwd) {
            label_wt_status_->setText("正向超时,开始反向...");
            QTimer::singleShot(1000, this, [this]{ wtStartStep(false); });
        } else {
            wt_phase_ = AT_ANALYZE;
            label_wt_status_->setText("反向超时,分析...");
            wtShowResults();
        }
    }
}

// ============================================================================
// 显示分析结果
// ============================================================================
void ManualControlWidget::wtShowResults()
{
    double step = spin_wt_step_->value();

    wt_result_fwd_ = atAnalyzeStep(wt_samples_fwd_, step);
    wt_result_rev_ = atAnalyzeStep(wt_samples_rev_, -step);

    QString unit = (wt_channel_ == WT_CH_DISTANCE) ? "m" : "°";

    QString text = QString(
        "===== 正向 =====\n"
        "上升时间: %1 秒    超调量: %2%    调节时间: %3 秒\n"
        "稳态误差: %4 %5    震荡: %6\n\n"
        "===== 反向 =====\n"
        "上升时间: %7 秒    超调量: %8%    调节时间: %9 秒\n"
        "稳态误差: %10 %11    震荡: %12")
        .arg(wt_result_fwd_.rise_time, 0, 'f', 2)
        .arg(wt_result_fwd_.overshoot, 0, 'f', 1)
        .arg(wt_result_fwd_.settle_time, 0, 'f', 2)
        .arg(wt_result_fwd_.steady_error, 0, 'f', 3).arg(unit)
        .arg(wt_result_fwd_.oscillating ? "是" : "否")
        .arg(wt_result_rev_.rise_time, 0, 'f', 2)
        .arg(wt_result_rev_.overshoot, 0, 'f', 1)
        .arg(wt_result_rev_.settle_time, 0, 'f', 2)
        .arg(wt_result_rev_.steady_error, 0, 'f', 3).arg(unit)
        .arg(wt_result_rev_.oscillating ? "是" : "否");

    label_wt_analysis_->setText(text);

    // 计算建议参数
    QString suggest;
    if (wt_channel_ == WT_CH_DISTANCE) {
        double cur_kp = spin_kp_x_->value();
        double cur_ki = spin_ki_x_->value();
        double cur_kd = spin_kd_x_->value();
        double new_kp = cur_kp, new_ki = cur_ki, new_kd = cur_kd;

        // 综合正反向结果取较差的
        double worst_overshoot = std::max(wt_result_fwd_.overshoot, wt_result_rev_.overshoot);
        double worst_rise = std::max(wt_result_fwd_.rise_time, wt_result_rev_.rise_time);
        double worst_error = std::max(wt_result_fwd_.steady_error, wt_result_rev_.steady_error);
        bool any_osc = wt_result_fwd_.oscillating || wt_result_rev_.oscillating;

        if (any_osc || worst_overshoot > 20) {
            new_kp *= 0.7; new_ki *= 0.7; new_kd *= 1.3;
        } else if (worst_overshoot > 10) {
            new_kp *= 0.85; new_kd *= 1.2;
        } else if (worst_rise > 5.0) {
            new_kp *= 1.3;
        } else if (worst_rise > 2.0) {
            new_kp *= 1.1;
        }
        if (worst_error > 0.1) new_ki *= 1.3;

        suggest = QString("建议距离参数: Kp=%1 Ki=%2 Kd=%3  (当前: Kp=%4 Ki=%5 Kd=%6)")
            .arg(new_kp, 0, 'f', 2).arg(new_ki, 0, 'f', 3).arg(new_kd, 0, 'f', 4)
            .arg(cur_kp, 0, 'f', 2).arg(cur_ki, 0, 'f', 3).arg(cur_kd, 0, 'f', 4);
    } else {
        double cur_kp = spin_kp_theta_->value();
        double cur_ki = spin_ki_theta_->value();
        double cur_kd = spin_kd_theta_->value();
        double new_kp = cur_kp, new_ki = cur_ki, new_kd = cur_kd;

        double worst_overshoot = std::max(wt_result_fwd_.overshoot, wt_result_rev_.overshoot);
        double worst_rise = std::max(wt_result_fwd_.rise_time, wt_result_rev_.rise_time);
        double worst_error = std::max(wt_result_fwd_.steady_error, wt_result_rev_.steady_error);
        bool any_osc = wt_result_fwd_.oscillating || wt_result_rev_.oscillating;

        if (any_osc || worst_overshoot > 20) {
            new_kp *= 0.7; new_ki *= 0.7; new_kd *= 1.3;
        } else if (worst_overshoot > 10) {
            new_kp *= 0.85; new_kd *= 1.2;
        } else if (worst_rise > 5.0) {
            new_kp *= 1.3;
        } else if (worst_rise > 2.0) {
            new_kp *= 1.1;
        }
        if (worst_error > 1.0) new_ki *= 1.3;

        suggest = QString("建议航向参数: Kp=%1 Ki=%2 Kd=%3  (当前: Kp=%4 Ki=%5 Kd=%6)")
            .arg(new_kp, 0, 'f', 2).arg(new_ki, 0, 'f', 3).arg(new_kd, 0, 'f', 4)
            .arg(cur_kp, 0, 'f', 2).arg(cur_ki, 0, 'f', 3).arg(cur_kd, 0, 'f', 4);
    }

    label_wt_suggest_->setText(suggest);

    // 收敛判定
    double err_threshold = (wt_channel_ == WT_CH_DISTANCE) ? 0.05 : 1.0;
    bool converged =
        wt_result_fwd_.overshoot < 5 && wt_result_rev_.overshoot < 5 &&
        !wt_result_fwd_.oscillating && !wt_result_rev_.oscillating &&
        wt_result_fwd_.steady_error < err_threshold &&
        wt_result_rev_.steady_error < err_threshold;

    if (converged) {
        label_wt_status_->setText("已收敛! 参数满足指标");
        label_wt_status_->setStyleSheet("color:#34C759;font-weight:bold;font-size:14px;");
        if (wt_auto_mode_) {
            wtApplyNewParams();
            wtStop();
            return;
        }
    } else {
        label_wt_status_->setText("分析完成 — 请确认或继续");
        label_wt_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
    }

    wt_phase_ = AT_WAIT_USER;
    btn_wt_apply_->setEnabled(true);
    btn_wt_auto_->setEnabled(true);

    // 自动继续模式
    if (wt_auto_mode_ && !converged) {
        wtApplyNewParams();
        if (wt_iteration_ >= spin_wt_max_iter_->value()) {
            label_wt_status_->setText("达到最大轮次,整定结束");
            label_wt_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            wtStop();
            return;
        }
        wt_iteration_++;
        label_wt_iter_->setText(QString("第 %1 轮").arg(wt_iteration_));
        wtStartStep(true);
    }
}

// ============================================================================
// 应用建议参数
// ============================================================================
void ManualControlWidget::wtApplyNewParams()
{
    if (wt_channel_ == WT_CH_DISTANCE) {
        double cur_kp = spin_kp_x_->value();
        double cur_ki = spin_ki_x_->value();
        double cur_kd = spin_kd_x_->value();
        double new_kp = cur_kp, new_ki = cur_ki, new_kd = cur_kd;

        double worst_overshoot = std::max(wt_result_fwd_.overshoot, wt_result_rev_.overshoot);
        double worst_rise = std::max(wt_result_fwd_.rise_time, wt_result_rev_.rise_time);
        double worst_error = std::max(wt_result_fwd_.steady_error, wt_result_rev_.steady_error);
        bool any_osc = wt_result_fwd_.oscillating || wt_result_rev_.oscillating;

        if (any_osc || worst_overshoot > 20) {
            new_kp *= 0.7; new_ki *= 0.7; new_kd *= 1.3;
        } else if (worst_overshoot > 10) {
            new_kp *= 0.85; new_kd *= 1.2;
        } else if (worst_rise > 5.0) {
            new_kp *= 1.3;
        } else if (worst_rise > 2.0) {
            new_kp *= 1.1;
        }
        if (worst_error > 0.1) new_ki *= 1.3;

        spin_kp_x_->setValue(new_kp);
        spin_ki_x_->setValue(new_ki);
        spin_kd_x_->setValue(new_kd);
    } else {
        double cur_kp = spin_kp_theta_->value();
        double cur_ki = spin_ki_theta_->value();
        double cur_kd = spin_kd_theta_->value();
        double new_kp = cur_kp, new_ki = cur_ki, new_kd = cur_kd;

        double worst_overshoot = std::max(wt_result_fwd_.overshoot, wt_result_rev_.overshoot);
        double worst_rise = std::max(wt_result_fwd_.rise_time, wt_result_rev_.rise_time);
        double worst_error = std::max(wt_result_fwd_.steady_error, wt_result_rev_.steady_error);
        bool any_osc = wt_result_fwd_.oscillating || wt_result_rev_.oscillating;

        if (any_osc || worst_overshoot > 20) {
            new_kp *= 0.7; new_ki *= 0.7; new_kd *= 1.3;
        } else if (worst_overshoot > 10) {
            new_kp *= 0.85; new_kd *= 1.2;
        } else if (worst_rise > 5.0) {
            new_kp *= 1.3;
        } else if (worst_rise > 2.0) {
            new_kp *= 1.1;
        }
        if (worst_error > 1.0) new_ki *= 1.3;

        spin_kp_theta_->setValue(new_kp);
        spin_ki_theta_->setValue(new_ki);
        spin_kd_theta_->setValue(new_kd);
    }

    // 触发发送行走参数
    btn_send_walk_params_->click();

    recordDiagEvent("WT", QString("应用行走参数 (轮次:%1)").arg(wt_iteration_));
}

// ============================================================================
// 停止整定
// ============================================================================
void ManualControlWidget::wtStop()
{
    wt_timer_->stop();
    wt_phase_ = AT_IDLE;
    wt_auto_mode_ = false;

    // 停车
    std_msgs::Float64 ws; ws.data = 0;
    pub_walk_state_.publish(ws);

    btn_wt_start_->setEnabled(true);
    btn_wt_stop_->setEnabled(false);
    btn_wt_apply_->setEnabled(false);
    btn_wt_auto_->setEnabled(false);
    combo_wt_channel_->setEnabled(true);

    if (!label_wt_status_->text().contains("收敛") &&
        !label_wt_status_->text().contains("最大轮次")) {
        label_wt_status_->setText("已停止");
        label_wt_status_->setStyleSheet("color:#AEAEB2;font-weight:bold;");
    }

    recordDiagEvent("WT", "行走整定停止");
}

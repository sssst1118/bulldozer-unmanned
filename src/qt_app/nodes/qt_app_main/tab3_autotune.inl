/**
 * @file tab3_autotune.inl
 * @brief 铲刀PID自动整定 — 阶跃响应法, 迭代收敛
 * @author dozer-dev
 * @date 2026-04-14
 *
 * 功能:
 *   - 对高度或角度通路执行 +/- 阶跃, 记录响应曲线
 *   - 分析上升时间、超调量、稳态误差、震荡
 *   - 根据分析结果自动调整PI参数
 *   - 支持手动逐轮确认和全自动迭代两种模式
 *
 * 集成到 Tab3 底部的 "自动整定" 分组中
 */

#include <QPainter>
#include <QPaintEvent>
#include <QComboBox>
#include <algorithm>
#include <cmath>

// ============================================================================
// 响应曲线绘图控件
// ============================================================================
class AutoTuneCurveWidget : public QWidget {
public:
    std::vector<ManualControlWidget::AutoTuneSample>* samples_up = nullptr;
    std::vector<ManualControlWidget::AutoTuneSample>* samples_dn = nullptr;
    double step_mm = 50;
    double baseline = 0;

    explicit AutoTuneCurveWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(220);
        setStyleSheet("background:white; border:1px solid #D1D1D6; border-radius:8px;");
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        int W = width() - 20, H = height() - 30;
        int ox = 10, oy = 10;

        // 背景
        p.fillRect(rect(), QColor("#F9FAFB"));
        p.setPen(QColor("#D1D1D6"));
        p.drawRect(ox, oy, W, H);

        // 无数据
        bool has_up = samples_up && !samples_up->empty();
        bool has_dn = samples_dn && !samples_dn->empty();
        if (!has_up && !has_dn) {
            p.setPen(QColor("#AEAEB2"));
            p.setFont(QFont("", 12));
            p.drawText(rect(), Qt::AlignCenter, "等待整定数据...");
            return;
        }

        // 计算时间和值范围
        double t_max = 0;
        double v_min = 1e9, v_max = -1e9;
        auto updateRange = [&](const std::vector<ManualControlWidget::AutoTuneSample>& s) {
            for (auto& pt : s) {
                if (pt.time_sec > t_max) t_max = pt.time_sec;
                if (pt.actual < v_min) v_min = pt.actual;
                if (pt.actual > v_max) v_max = pt.actual;
                if (pt.target < v_min) v_min = pt.target;
                if (pt.target > v_max) v_max = pt.target;
            }
        };
        if (has_up) updateRange(*samples_up);
        if (has_dn) {
            // 负阶跃时间偏移到正阶跃之后
            double t_offset = has_up ? samples_up->back().time_sec + 0.5 : 0;
            for (auto& pt : *samples_dn) {
                double t = pt.time_sec + t_offset;
                if (t > t_max) t_max = t;
                if (pt.actual < v_min) v_min = pt.actual;
                if (pt.actual > v_max) v_max = pt.actual;
                if (pt.target < v_min) v_min = pt.target;
                if (pt.target > v_max) v_max = pt.target;
            }
        }

        if (t_max < 0.1) t_max = 1.0;
        double v_range = v_max - v_min;
        if (v_range < 1.0) { v_range = 10.0; v_min -= 5; v_max += 5; }
        v_min -= v_range * 0.1;
        v_max += v_range * 0.1;
        v_range = v_max - v_min;

        auto tx = [&](double t) -> int { return ox + (int)(t / t_max * W); };
        auto ty = [&](double v) -> int { return oy + H - (int)((v - v_min) / v_range * H); };

        // 网格线
        p.setPen(QPen(QColor("#E0E0E5"), 1, Qt::DotLine));
        for (int i = 1; i < 4; i++) {
            int y = oy + H * i / 4;
            p.drawLine(ox, y, ox + W, y);
        }
        for (int i = 1; i < 4; i++) {
            int x = ox + W * i / 4;
            p.drawLine(x, oy, x, oy + H);
        }

        // 绘制曲线的lambda
        auto drawCurve = [&](const std::vector<ManualControlWidget::AutoTuneSample>& s,
                             double t_off, QColor target_color, QColor actual_color) {
            if (s.size() < 2) return;
            // 目标线 (虚线)
            p.setPen(QPen(target_color, 2, Qt::DashLine));
            for (size_t i = 1; i < s.size(); i++) {
                p.drawLine(tx(s[i-1].time_sec + t_off), ty(s[i-1].target),
                           tx(s[i].time_sec + t_off), ty(s[i].target));
            }
            // 实际线 (实线)
            p.setPen(QPen(actual_color, 2));
            for (size_t i = 1; i < s.size(); i++) {
                p.drawLine(tx(s[i-1].time_sec + t_off), ty(s[i-1].actual),
                           tx(s[i].time_sec + t_off), ty(s[i].actual));
            }
        };

        if (has_up) drawCurve(*samples_up, 0, QColor("#FF9500"), QColor("#007AFF"));
        if (has_dn) {
            double t_off = has_up ? samples_up->back().time_sec + 0.5 : 0;
            drawCurve(*samples_dn, t_off, QColor("#FF9500"), QColor("#34C759"));
        }

        // 图例
        int lx = ox + 8, ly = oy + 12;
        p.setFont(QFont("", 9));
        p.setPen(QPen(QColor("#FF9500"), 2, Qt::DashLine));
        p.drawLine(lx, ly, lx + 20, ly);
        p.setPen(QColor("#000000"));
        p.drawText(lx + 24, ly + 4, "目标");
        ly += 16;
        p.setPen(QPen(QColor("#007AFF"), 2));
        p.drawLine(lx, ly, lx + 20, ly);
        p.setPen(QColor("#000000"));
        p.drawText(lx + 24, ly + 4, "实际(+)");
        if (has_dn) {
            ly += 16;
            p.setPen(QPen(QColor("#34C759"), 2));
            p.drawLine(lx, ly, lx + 20, ly);
            p.setPen(QColor("#000000"));
            p.drawText(lx + 24, ly + 4, "实际(-)");
        }

        // 时间轴标签
        p.setPen(QColor("#AEAEB2"));
        p.setFont(QFont("", 8));
        p.drawText(ox, oy + H + 2, W, 15, Qt::AlignLeft, "0s");
        p.drawText(ox, oy + H + 2, W, 15, Qt::AlignCenter,
                   QString("%1s").arg(t_max / 2, 0, 'f', 1));
        p.drawText(ox, oy + H + 2, W, 15, Qt::AlignRight,
                   QString("%1s").arg(t_max, 0, 'f', 1));
    }
};

// ============================================================================
// Tab3 自动整定UI (在 setupTab3_PIDTuning 末尾调用)
// ============================================================================
void ManualControlWidget::setupTab3_AutoTune(QVBoxLayout* layout)
{
    auto* atG = makeGroup("铲刀 PID 自动整定");
    auto* atL = new QVBoxLayout(atG);

    //=== 参数行 ===
    auto* paramRow = new QHBoxLayout();

    paramRow->addWidget(new QLabel("通道:"));
    // iOS SegmentedControl 2 段: 高度 / 角度 — 铲刀自动整定通道
    combo_at_channel_ = new SegmentedControl({"高度", "角度"});
    combo_at_channel_->setFixedWidth(140);
    paramRow->addWidget(combo_at_channel_);

    paramRow->addWidget(new QLabel("阶跃幅度(mm):"));
    spin_at_step_mm_ = makeSpin(5, 200, 50, 5, 0);
    spin_at_step_mm_->setFixedWidth(80);
    paramRow->addWidget(spin_at_step_mm_);

    paramRow->addWidget(new QLabel("稳定时间(秒):"));
    spin_at_settle_sec_ = makeSpin(1, 30, 5, 0.5, 1);
    spin_at_settle_sec_->setFixedWidth(80);
    paramRow->addWidget(spin_at_settle_sec_);

    paramRow->addWidget(new QLabel("最大轮次:"));
    spin_at_max_iter_ = new QSpinBox();
    spin_at_max_iter_->setRange(1, 50);
    spin_at_max_iter_->setValue(10);
    spin_at_max_iter_->setFixedWidth(60);
    paramRow->addWidget(spin_at_max_iter_);

    paramRow->addStretch();
    atL->addLayout(paramRow);

    //=== 按钮行 ===
    auto* btnRow = new QHBoxLayout();
    // IOSButton::Primary — 开始标定 (iOS 蓝)
    btn_at_start_ = new IOSButton("开始标定", IOSButton::Primary);
    btn_at_start_->setFixedHeight(40);
    btnRow->addWidget(btn_at_start_);

    // IOSButton::Warn — 应用并继续 (iOS 橙)
    btn_at_apply_ = new IOSButton("应用并继续", IOSButton::Warn);
    btn_at_apply_->setFixedHeight(40);
    btn_at_apply_->setEnabled(false);
    btnRow->addWidget(btn_at_apply_);

    // IOSButton::Success — 自动继续 (iOS 绿)
    btn_at_auto_ = new IOSButton("自动继续", IOSButton::Success);
    btn_at_auto_->setFixedHeight(40);
    btn_at_auto_->setEnabled(false);
    btnRow->addWidget(btn_at_auto_);

    // IOSButton::Danger — 停止 (iOS 红)
    btn_at_stop_ = new IOSButton("停止", IOSButton::Danger);
    btn_at_stop_->setFixedHeight(40);
    btn_at_stop_->setEnabled(false);
    btnRow->addWidget(btn_at_stop_);

    atL->addLayout(btnRow);

    //=== 状态行 ===
    auto* statusRow = new QHBoxLayout();
    label_at_status_ = new QLabel("就绪");
    label_at_status_->setFont(QFont("", 12, QFont::Bold));
    label_at_status_->setStyleSheet("color:#AEAEB2;");
    statusRow->addWidget(label_at_status_);
    statusRow->addStretch();
    label_at_iter_ = new QLabel("");
    label_at_iter_->setFont(MONO_FONT);
    statusRow->addWidget(label_at_iter_);
    atL->addLayout(statusRow);

    //=== 响应曲线 ===
    auto* curveW = new AutoTuneCurveWidget(this);
    at_curve_widget_ = curveW;
    atL->addWidget(at_curve_widget_);

    //=== 分析结果 ===
    label_at_analysis_ = new QLabel("");
    label_at_analysis_->setFont(MONO_FONT);
    label_at_analysis_->setWordWrap(true);
    label_at_analysis_->setStyleSheet("background:#F9FAFB;border-radius:8px;padding:8px;");
    atL->addWidget(label_at_analysis_);

    //=== 建议参数 ===
    label_at_suggest_ = new QLabel("");
    label_at_suggest_->setFont(QFont("", 11, QFont::Bold));
    label_at_suggest_->setWordWrap(true);
    label_at_suggest_->setStyleSheet("color:#007AFF;padding:4px;");
    atL->addWidget(label_at_suggest_);

    layout->addWidget(atG);

    //=== 采样定时器 (50Hz) ===
    at_timer_ = new QTimer(this);
    connect(at_timer_, &QTimer::timeout, this, &ManualControlWidget::atSampleTick);

    //=== 按钮连接 ===
    connect(btn_at_start_, &QPushButton::clicked, [this]{
        if (at_phase_ != AT_IDLE) return;

        // [Fix-v20] 主开关必须关闭, 否则 decision 会覆盖 ref 话题
        if (node_main_switch_on_) {
            label_at_status_->setText("请先关闭主开关 (Tab2)");
            label_at_status_->setStyleSheet("color:#DC2626;font-weight:bold;");
            return;
        }

        at_channel_ = (combo_at_channel_->selectedIndex() == 0) ? AT_CH_HEIGHT : AT_CH_THETA;
        at_iteration_ = 0;
        at_auto_mode_ = false;

        // [Fix-v20] 保存当前实际位置, 整定结束后恢复
        at_original_ref_height_ = (mold_left_mm_ + mold_right_mm_) / 2.0 / 1000.0; // mm → m
        at_original_ref_angle_  = mold_imu_roll_deg_;

        btn_at_start_->setEnabled(false);
        btn_at_stop_->setEnabled(true);
        btn_at_apply_->setEnabled(false);
        btn_at_auto_->setEnabled(false);
        combo_at_channel_->setEnabled(false);

        // 开始第一轮: 先正阶跃
        at_iteration_++;
        label_at_iter_->setText(QString("第 %1 轮").arg(at_iteration_));
        atStartStep(true);
    });

    connect(btn_at_apply_, &QPushButton::clicked, [this]{
        if (at_phase_ != AT_WAIT_USER) return;
        atApplyNewParams();

        // 检查是否达到最大轮次
        if (at_iteration_ >= spin_at_max_iter_->value()) {
            label_at_status_->setText("达到最大轮次,整定结束");
            label_at_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            atStop();
            return;
        }

        // 开始下一轮
        at_iteration_++;
        label_at_iter_->setText(QString("第 %1 轮").arg(at_iteration_));
        atStartStep(true);
    });

    connect(btn_at_auto_, &QPushButton::clicked, [this]{
        if (at_phase_ != AT_WAIT_USER) return;
        at_auto_mode_ = true;
        btn_at_auto_->setEnabled(false);
        btn_at_apply_->setEnabled(false);
        // 应用参数并继续
        atApplyNewParams();

        if (at_iteration_ >= spin_at_max_iter_->value()) {
            label_at_status_->setText("达到最大轮次,整定结束");
            label_at_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            atStop();
            return;
        }

        at_iteration_++;
        label_at_iter_->setText(QString("第 %1 轮").arg(at_iteration_));
        atStartStep(true);
    });

    connect(btn_at_stop_, &QPushButton::clicked, [this]{
        atStop();
    });
}

// ============================================================================
// 开始一次阶跃
// ============================================================================
void ManualControlWidget::atStartStep(bool up)
{
    double step_mm = spin_at_step_mm_->value();
    double step_m  = step_mm / 1000.0;

    if (up) {
        at_phase_ = AT_STEP_UP;
        at_samples_up_.clear();
        at_result_up_ = {};
        at_samples_dn_.clear();
        at_result_dn_ = {};
        label_at_status_->setText("正阶跃中...");
        label_at_status_->setStyleSheet("color:#007AFF;font-weight:bold;");
    } else {
        at_phase_ = AT_STEP_DN;
        at_samples_dn_.clear();
        at_result_dn_ = {};
        label_at_status_->setText("负阶跃中...");
        label_at_status_->setStyleSheet("color:#34C759;font-weight:bold;");
    }

    // 获取当前实际值作为基准
    if (at_channel_ == AT_CH_HEIGHT) {
        // 从mold_debug获取的实际中心高度
        // mold_e_height_ = ref - actual, 所以 actual ≈ ref - e
        // 但更准确的是直接用铲刀传感器数据
        at_baseline_ = (mold_left_mm_ + mold_right_mm_) / 2.0; // mm
        at_target_val_ = at_baseline_ + (up ? step_mm : -step_mm);

        // 发布新的ref_height (转换为m)
        std_msgs::Float64 msg;
        msg.data = at_target_val_ / 1000.0;
        pub_ref_height_.publish(msg);
    } else {
        at_baseline_ = mold_imu_roll_deg_;  // [Fix-v19] 见上方注释
        double step_deg = step_mm / 10.0; // 角度通路: mm参数当0.1度用
        at_target_val_ = at_baseline_ + (up ? step_deg : -step_deg);

        std_msgs::Float64 msg;
        msg.data = at_target_val_;
        pub_ref_angle_.publish(msg);
    }

    at_step_start_time_ = ros::Time::now().toSec();
    at_settle_deadline_ = 0; // 还没开始等稳定

    // 确保铲刀控制使能
    {
        std_msgs::Float64 flag;
        flag.data = 1.0;
        pub_mold_ctrl_flag_.publish(flag);
    }

    // 启动采样定时器 50Hz
    at_timer_->start(20);

    recordDiagEvent("AT", QString("阶跃 %1 → 目标 %2 (通道:%3 轮次:%4)")
        .arg(up ? "+" : "-")
        .arg(at_target_val_, 0, 'f', 1)
        .arg(at_channel_ == AT_CH_HEIGHT ? "高度" : "角度")
        .arg(at_iteration_));
}

// ============================================================================
// 采样定时器回调 (50Hz)
// ============================================================================
void ManualControlWidget::atSampleTick()
{
    double now = ros::Time::now().toSec();
    double t = now - at_step_start_time_;

    AutoTuneSample s;
    s.time_sec = t;

    if (at_channel_ == AT_CH_HEIGHT) {
        s.target = at_target_val_;  // mm
        s.actual = (mold_left_mm_ + mold_right_mm_) / 2.0; // mm
        s.control_out = mold_u_height_;
    } else {
        s.target = at_target_val_;  // deg
        s.actual = mold_imu_roll_deg_;  // [Fix-v19] 用铲刀 IMU 横滚 (原 imu_roll_deg_ 来自不存在的旧IMU)
        s.control_out = mold_u_theta_;
    }

    bool is_up = (at_phase_ == AT_STEP_UP || at_phase_ == AT_WAIT_UP);
    auto& samples = is_up ? at_samples_up_ : at_samples_dn_;
    samples.push_back(s);

    // 更新曲线
    auto* cw = static_cast<AutoTuneCurveWidget*>(at_curve_widget_);
    cw->samples_up = &at_samples_up_;
    cw->samples_dn = &at_samples_dn_;
    cw->step_mm = spin_at_step_mm_->value();
    cw->baseline = at_baseline_;
    at_curve_widget_->update();

    // 判断是否稳定: 误差小于阶跃幅度的5%持续settle_time秒
    double step_size = spin_at_step_mm_->value();
    if (at_channel_ == AT_CH_THETA) step_size = step_size / 10.0;
    double error = std::abs(s.actual - s.target);
    double threshold = step_size * 0.05;
    if (threshold < 1.0) threshold = 1.0; // 至少1mm或0.1度

    double settle_sec = spin_at_settle_sec_->value();

    if (error < threshold) {
        if (at_settle_deadline_ == 0) {
            at_settle_deadline_ = now + settle_sec;
            if (is_up) at_phase_ = AT_WAIT_UP;
            else       at_phase_ = AT_WAIT_DN;
            label_at_status_->setText(is_up ? "正阶跃 等待稳定..." : "负阶跃 等待稳定...");
        }
        if (now >= at_settle_deadline_) {
            // 稳定了
            at_timer_->stop();
            if (is_up) {
                // 正阶跃完成,开始负阶跃
                atStartStep(false);
            } else {
                // 负阶跃完成,分析
                at_phase_ = AT_ANALYZE;
                atShowResults();
            }
            return;
        }
    } else {
        at_settle_deadline_ = 0; // 还没稳定,重置
        if (is_up) at_phase_ = AT_STEP_UP;
        else       at_phase_ = AT_STEP_DN;
    }

    // 超时保护: 30秒还没稳定
    if (t > 30.0) {
        at_timer_->stop();
        if (is_up) {
            // 正阶跃超时,也开始负阶跃
            label_at_status_->setText("正阶跃超时,开始负阶跃...");
            atStartStep(false);
        } else {
            at_phase_ = AT_ANALYZE;
            label_at_status_->setText("负阶跃超时,分析...");
            atShowResults();
        }
    }
}

// ============================================================================
// 分析阶跃响应
// ============================================================================
ManualControlWidget::StepAnalysis ManualControlWidget::atAnalyzeStep(
    const std::vector<AutoTuneSample>& samples, double step_size)
{
    StepAnalysis r;
    if (samples.size() < 10) return r;

    double baseline = samples.front().actual;
    double target = samples.front().target;
    double direction = (target > baseline) ? 1.0 : -1.0;
    double step_abs = std::abs(step_size);

    // 上升时间: 从0到达90%阶跃幅度的时间
    double threshold_90 = baseline + direction * step_abs * 0.9;
    for (auto& s : samples) {
        bool reached = (direction > 0) ? (s.actual >= threshold_90) : (s.actual <= threshold_90);
        if (reached) { r.rise_time = s.time_sec; break; }
    }
    if (r.rise_time == 0) r.rise_time = samples.back().time_sec; // 没到

    // 超调量: 超过目标的最大值
    double max_overshoot = 0;
    for (auto& s : samples) {
        double overshoot = direction * (s.actual - target);
        if (overshoot > max_overshoot) max_overshoot = overshoot;
    }
    r.overshoot = (step_abs > 0.01) ? (max_overshoot / step_abs * 100.0) : 0;

    // 稳态误差: 最后1秒的平均误差
    double sum_err = 0;
    int count = 0;
    double t_end = samples.back().time_sec;
    for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
        if (it->time_sec < t_end - 1.0) break;
        sum_err += std::abs(it->actual - it->target);
        count++;
    }
    r.steady_error = (count > 0) ? sum_err / count : 0;

    // 调节时间: 最后一次误差超过5%阶跃幅度的时间
    double settle_threshold = step_abs * 0.05;
    if (settle_threshold < 1.0) settle_threshold = 1.0;
    r.settle_time = 0;
    for (auto& s : samples) {
        if (std::abs(s.actual - s.target) > settle_threshold) {
            r.settle_time = s.time_sec;
        }
    }

    // 震荡检测: 实际值穿越目标线超过2次
    int crossings = 0;
    for (size_t i = 1; i < samples.size(); i++) {
        double prev_err = samples[i-1].actual - target;
        double curr_err = samples[i].actual - target;
        if (prev_err * curr_err < 0) crossings++;
    }
    r.oscillating = (crossings > 2);

    return r;
}

// ============================================================================
// 计算新PI参数 (公共逻辑, 去重)
// ============================================================================
ManualControlWidget::PIParams ManualControlWidget::atCalcNewParams()
{
    double kp_up, kp_dn, ki_up, ki_dn;
    double err_threshold;

    if (at_channel_ == AT_CH_HEIGHT) {
        kp_up = spin_kp_h_up_->value();
        kp_dn = spin_kp_h_dn_->value();
        ki_up = spin_ki_h_up_->value();
        ki_dn = spin_ki_h_dn_->value();
        err_threshold = 3.0;  // mm
    } else {
        kp_up = spin_kp_t_up_->value();
        kp_dn = spin_kp_t_dn_->value();
        ki_up = spin_ki_t_up_->value();
        ki_dn = spin_ki_t_dn_->value();
        err_threshold = 0.5;  // deg
    }

    // 正阶跃 → 调升方向参数
    auto adjustOne = [](double& kp, double& ki, const StepAnalysis& r, double err_thr) {
        if (r.oscillating || r.overshoot > 20) {
            kp *= 0.7; ki *= 0.7;
        } else if (r.overshoot > 10) {
            kp *= 0.85;
        } else if (r.rise_time > 3.0) {
            kp *= 1.3;
        } else if (r.rise_time > 1.5) {
            kp *= 1.1;
        }
        if (r.steady_error > err_thr) {
            ki *= 1.3;
        }
    };

    adjustOne(kp_up, ki_up, at_result_up_, err_threshold);
    adjustOne(kp_dn, ki_dn, at_result_dn_, err_threshold);

    return {kp_up, kp_dn, ki_up, ki_dn};
}

// ============================================================================
// 显示分析结果
// ============================================================================
void ManualControlWidget::atShowResults()
{
    double step_size = spin_at_step_mm_->value();
    if (at_channel_ == AT_CH_THETA) step_size = step_size / 10.0;

    at_result_up_ = atAnalyzeStep(at_samples_up_, step_size);
    at_result_dn_ = atAnalyzeStep(at_samples_dn_, -step_size);

    QString unit = (at_channel_ == AT_CH_HEIGHT) ? "mm" : "°";

    QString text = QString(
        "===== 正阶跃(+) =====\n"
        "上升时间: %1 秒    超调量: %2%    调节时间: %3 秒\n"
        "稳态误差: %4 %5    震荡: %6\n\n"
        "===== 负阶跃(-) =====\n"
        "上升时间: %7 秒    超调量: %8%    调节时间: %9 秒\n"
        "稳态误差: %10 %11    震荡: %12")
        .arg(at_result_up_.rise_time, 0, 'f', 2)
        .arg(at_result_up_.overshoot, 0, 'f', 1)
        .arg(at_result_up_.settle_time, 0, 'f', 2)
        .arg(at_result_up_.steady_error, 0, 'f', 2).arg(unit)
        .arg(at_result_up_.oscillating ? "是" : "否")
        .arg(at_result_dn_.rise_time, 0, 'f', 2)
        .arg(at_result_dn_.overshoot, 0, 'f', 1)
        .arg(at_result_dn_.settle_time, 0, 'f', 2)
        .arg(at_result_dn_.steady_error, 0, 'f', 2).arg(unit)
        .arg(at_result_dn_.oscillating ? "是" : "否");

    label_at_analysis_->setText(text);

    // [Fix-v20] 用公共函数计算建议参数 (去重)
    PIParams np = atCalcNewParams();

    double cur_kp_up, cur_kp_dn, cur_ki_up, cur_ki_dn;
    if (at_channel_ == AT_CH_HEIGHT) {
        cur_kp_up = spin_kp_h_up_->value(); cur_kp_dn = spin_kp_h_dn_->value();
        cur_ki_up = spin_ki_h_up_->value(); cur_ki_dn = spin_ki_h_dn_->value();
    } else {
        cur_kp_up = spin_kp_t_up_->value(); cur_kp_dn = spin_kp_t_dn_->value();
        cur_ki_up = spin_ki_t_up_->value(); cur_ki_dn = spin_ki_t_dn_->value();
    }

    QString ch_name = (at_channel_ == AT_CH_HEIGHT) ? "高度" : "角度";
    QString suggest = QString("建议%9参数: Kp升=%1 Kp降=%2 Ki升=%3 Ki降=%4  (当前: Kp升=%5 Kp降=%6 Ki升=%7 Ki降=%8)")
        .arg(np.kp_up, 0, 'f', 2).arg(np.kp_dn, 0, 'f', 2)
        .arg(np.ki_up, 0, 'f', 3).arg(np.ki_dn, 0, 'f', 3)
        .arg(cur_kp_up, 0, 'f', 2).arg(cur_kp_dn, 0, 'f', 2)
        .arg(cur_ki_up, 0, 'f', 3).arg(cur_ki_dn, 0, 'f', 3)
        .arg(ch_name);

    label_at_suggest_->setText(suggest);

    // 响应指标收敛 (超调<5%, 稳态误差<2, 无震荡)
    bool response_converged =
        at_result_up_.overshoot < 5 && at_result_dn_.overshoot < 5 &&
        !at_result_up_.oscillating && !at_result_dn_.oscillating &&
        at_result_up_.steady_error < 2 && at_result_dn_.steady_error < 2;

    // [Fix-v20] 参数收敛: 新旧参数变化量 < 2% 则停止, 防止来回震荡
    auto relChange = [](double old_v, double new_v) {
        return (std::abs(old_v) > 0.001) ? std::abs(new_v - old_v) / std::abs(old_v) : 0.0;
    };
    bool param_converged =
        relChange(cur_kp_up, np.kp_up) < 0.02 && relChange(cur_kp_dn, np.kp_dn) < 0.02 &&
        relChange(cur_ki_up, np.ki_up) < 0.02 && relChange(cur_ki_dn, np.ki_dn) < 0.02;

    bool converged = response_converged || param_converged;

    if (converged) {
        QString reason = response_converged ? "响应指标满足" : "参数已稳定(变化<2%)";
        label_at_status_->setText(QString("已收敛! %1").arg(reason));
        label_at_status_->setStyleSheet("color:#34C759;font-weight:bold;font-size:14px;");
        if (at_auto_mode_) {
            atApplyNewParams();
            atStop();
            return;
        }
    } else {
        label_at_status_->setText("分析完成 — 请确认或继续");
        label_at_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
    }

    at_phase_ = AT_WAIT_USER;
    btn_at_apply_->setEnabled(true);
    btn_at_auto_->setEnabled(true);

    // 自动继续模式
    if (at_auto_mode_ && !converged) {
        atApplyNewParams();
        if (at_iteration_ >= spin_at_max_iter_->value()) {
            label_at_status_->setText("达到最大轮次,整定结束");
            label_at_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
            atStop();
            return;
        }
        at_iteration_++;
        label_at_iter_->setText(QString("第 %1 轮").arg(at_iteration_));
        atStartStep(true);
    }
}

// ============================================================================
// 应用建议参数到SpinBox并发送
// ============================================================================
void ManualControlWidget::atApplyNewParams()
{
    // [Fix-v20] 用公共函数计算 (去重, 和 atShowResults 保证一致)
    PIParams np = atCalcNewParams();

    if (at_channel_ == AT_CH_HEIGHT) {
        spin_kp_h_up_->setValue(np.kp_up);
        spin_kp_h_dn_->setValue(np.kp_dn);
        spin_ki_h_up_->setValue(np.ki_up);
        spin_ki_h_dn_->setValue(np.ki_dn);
    } else {
        spin_kp_t_up_->setValue(np.kp_up);
        spin_kp_t_dn_->setValue(np.kp_dn);
        spin_ki_t_up_->setValue(np.ki_up);
        spin_ki_t_dn_->setValue(np.ki_dn);
    }

    // 触发"发送铲刀参数"
    btn_send_mold_params_->click();

    recordDiagEvent("AT", QString("应用新参数 (轮次:%1)").arg(at_iteration_));
}

// ============================================================================
// 停止整定
// ============================================================================
void ManualControlWidget::atStop()
{
    at_timer_->stop();

    // [Fix-v20] 恢复整定前的原始目标值, 避免铲刀停在阶跃位置
    if (at_phase_ != AT_IDLE) {
        if (at_channel_ == AT_CH_HEIGHT) {
            std_msgs::Float64 msg;
            msg.data = at_original_ref_height_;
            pub_ref_height_.publish(msg);
        } else {
            std_msgs::Float64 msg;
            msg.data = at_original_ref_angle_;
            pub_ref_angle_.publish(msg);
        }
    }

    at_phase_ = AT_IDLE;
    at_auto_mode_ = false;

    btn_at_start_->setEnabled(true);
    btn_at_stop_->setEnabled(false);
    btn_at_apply_->setEnabled(false);
    btn_at_auto_->setEnabled(false);
    combo_at_channel_->setEnabled(true);

    if (label_at_status_->text() == "就绪")
        return; // 没开始过

    if (!label_at_status_->text().contains("收敛") &&
        !label_at_status_->text().contains("最大轮次")) {
        label_at_status_->setText("已停止");
        label_at_status_->setStyleSheet("color:#AEAEB2;font-weight:bold;");
    }

    recordDiagEvent("AT", "整定停止");
}

/**
 * @file manual_control_widget.h
 * @brief 推土机无人驾驶控制台
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 布局: 左侧状态栏(固定) + 右侧Tab区
 *
 * Tab1 直接CAN控制      — 绕过规控, 直驱CAN
 * Tab2 节点功能测试      — 通过decision/control节点
 * Tab3 PID调参           — 行走+铲刀PID参数在线调节
 * Tab4 整车状态          — 发动机/变速箱/电流/姿态
 * Tab5 规划参数          — 路径规划增益/容差/地图参数
 * Tab6 车辆配置          — 默认参数模板, 保存/加载, 切换车辆
 * Tab7 数据录制          — rosbag一键录制
 * Tab8 通信诊断          — 话题频率/超时检测
 * Tab9 诊断快照          — 环形缓冲+手动触发保存, 全链路数据记录
 */
#ifndef MANUAL_CONTROL_WIDGET_H
#define MANUAL_CONTROL_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QAbstractButton>
#include <QCheckBox>
#include <QTimer>
#include <QGridLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QTextEdit>
#include <QListWidget>
#include <QComboBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QLineEdit>
#include <QProcess>
#include <QMap>
#include <QElapsedTimer>
#include <QDialog>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QShortcut>
#include <QPropertyAnimation>
#include <QProgressBar>
#include <QStackedWidget>
#include <QScrollArea>
#include <deque>

/**
 * @brief 可点击参数标签
 * @details 继承 QLabel，鼠标悬停显示 ToolTip 简述，点击弹出 QDialog 详细说明。
 *          用于替换 Tab3 中所有普通参数标签，提升调参可读性。
 */
class ParamLabel : public QLabel {
    Q_OBJECT
public:
    /**
     * @param text     标签显示文字，如 "Kp:"
     * @param tooltip  鼠标悬停的简短说明（一句话）
     * @param detail   点击后弹窗的详细 HTML 说明
     */
    ParamLabel(const QString& text,
               const QString& tooltip,
               const QString& detail,
               QWidget* parent = nullptr)
        : QLabel(text, parent), detail_(detail)
    {
        setToolTip(tooltip);
        setToolTipDuration(8000);
        setCursor(Qt::PointingHandCursor);
        setStyleSheet(
            "QLabel {"
            "  color: #007AFF;"                      // 特斯拉蓝
            "  text-decoration: underline dotted;"
            "}"
            "QLabel:hover { color: #0062D4; }"       // hover 深一级
        );
    }

protected:
    void mousePressEvent(QMouseEvent*) override
    {
        auto* dlg = new QDialog(this);
        dlg->setWindowTitle(text().remove(':').trimmed());
        dlg->setMinimumWidth(440);
        dlg->setStyleSheet("QDialog { background: #FFFFFF; }"
                           "QLabel  { color: #000000; font-size: 13px; line-height: 1.7; }"
                           "QPushButton { background:#007AFF; color:white; border-radius:8px;"
                           "             padding:8px 24px; font-weight:500; border:none; }"
                           "QPushButton:hover { background:#0062D4; }"
                           "QPushButton:pressed { background:#0055B3; }");
        auto* lay = new QVBoxLayout(dlg);
        lay->setContentsMargins(20, 16, 20, 16);
        lay->setSpacing(12);

        auto* lbl = new QLabel(detail_, dlg);
        lbl->setWordWrap(true);
        lbl->setTextFormat(Qt::RichText);
        lay->addWidget(lbl);

        auto* btn = new QPushButton("关闭", dlg);
        connect(btn, &QPushButton::clicked, dlg, &QDialog::accept);
        lay->addWidget(btn, 0, Qt::AlignRight);

        dlg->setAttribute(Qt::WA_DeleteOnClose);
        dlg->exec();
    }

private:
    QString detail_;
};

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>

class GridMapWidget;
class RvizPerceptionWidget;

//==============================================================================
// Tab4 整车状态 — 自绘仪表控件
//==============================================================================

/**
 * @brief 圆形仪表
 * @details 270° 进度弧 + 中心大号数字 + 单位 + 底部标签。
 *          颜色随阈值自动切换: 蓝(正常) → 橙(警告) → 红(危险)。
 *          支持"低值危险"模式(如燃油),通过 setThresholds 的 lowIsDanger 参数切换。
 *          setValue 自动用 QVariantAnimation 平滑过渡 (300ms OutCubic),避免数值突变。
 */
class CircularGauge : public QWidget {
    Q_OBJECT
public:
    CircularGauge(const QString& label, const QString& unit, QWidget* parent = nullptr);
    void setRange(double mn, double mx) { min_ = mn; max_ = mx; update(); }
    void setThresholds(double warn, double danger, bool lowIsDanger = false) {
        warn_ = warn; danger_ = danger; low_is_danger_ = lowIsDanger; update();
    }
    void setValue(double v);                        ///< 带动画平滑过渡
    void setDecimals(int d) { decimals_ = d; update(); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString label_, unit_;
    double min_ = 0, max_ = 100;
    double warn_ = 80, danger_ = 95;
    bool low_is_danger_ = false;
    double value_ = 0;                              ///< 当前绘制值 (动画中间值)
    double target_value_ = 0;                       ///< 目标值 (setValue 入参)
    int decimals_ = 0;
    class QVariantAnimation* anim_ = nullptr;
};

/**
 * @brief 水平仪条
 * @details 用于显示 Pitch / Roll 等双向角度, 中心零位, 左负右正。
 *          指针颜色随偏离中心程度变化: 蓝 → 橙 → 红。
 *          setValue 带动画平滑过渡 (300ms OutCubic)。
 */
class LevelBar : public QWidget {
    Q_OBJECT
public:
    LevelBar(const QString& label, double range = 30, QWidget* parent = nullptr);
    void setValue(double v);                        ///< 带动画平滑过渡

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString label_;
    double value_ = 0;                              ///< 当前绘制值 (动画中间值)
    double target_value_ = 0;
    double range_ = 30;
    class QVariantAnimation* anim_ = nullptr;
};

/**
 * @brief 侧边栏状态圆点 (10×10, 自绘, 颜色动画过渡)
 * @details 替代原先 QLabel+setStyleSheet 瞬时切色. 状态变化时用
 *   animateColor 对 dotColor 300ms 平滑过渡, 视觉更丝滑.
 */
class SideDot : public QWidget {
    Q_OBJECT
    Q_PROPERTY(QColor dotColor READ dotColor WRITE setDotColor)
public:
    explicit SideDot(QWidget* parent = nullptr);
    QColor dotColor() const { return dot_color_; }
    void setDotColor(const QColor& c) { dot_color_ = c; update(); }
    /// 启动颜色平滑过渡 (InOutQuad 300ms)
    void animateTo(const QColor& c);
protected:
    void paintEvent(QPaintEvent*) override;
private:
    QColor dot_color_;
};

/**
 * @brief 状态胶囊
 * @details 左圆灯 + 标签 + 值文本, 适合二元或多态状态 (手刹/液压锁/手动自动/制动阀)。
 */
class StatusPill : public QWidget {
    Q_OBJECT
public:
    enum State { PILL_OFF, PILL_ON, PILL_NORMAL, PILL_WARN, PILL_DANGER };
    StatusPill(const QString& label, QWidget* parent = nullptr)
        : QWidget(parent), label_(label) {
        setMinimumSize(130, 48);
        setMaximumHeight(52);
    }
    void setState(State s, const QString& valueText = "") {
        state_ = s; value_ = valueText; update();
    }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString label_, value_ = "--";
    State state_ = PILL_OFF;
};

/**
 * @brief 增强型浮点数输入框
 * @details 继承 QDoubleSpinBox, 不改变接口, 叠加两类体验改进:
 *   - **底部范围色带**: 可视化显示当前值在 [min, max] 中的位置;
 *     可选标注推荐范围(绿色块)和临界范围(红色块);
 *     蓝色圆点指示当前值。
 *   - **修饰键滚轮加速**: Shift+滚轮 ×10 粗调, Alt+滚轮 ÷10 细调,
 *     配合 ManualControlWidget 的"无焦点不响应滚轮"过滤器。
 *   API 完全兼容 QDoubleSpinBox, 因此 makeSpin 只需改 `new SmartSpinBox`
 *   即可全局替换, 其余调用点零改动。
 */
class SmartSpinBox : public QDoubleSpinBox {
    Q_OBJECT
    // Focus 光晕动画的 property (规则七: 特斯拉风浅蓝内圈而非发光)
    Q_PROPERTY(double focusRingOpacity READ focusRingOpacity WRITE setFocusRingOpacity)
public:
    explicit SmartSpinBox(QWidget* parent = nullptr);

    /// 设置推荐范围 (色带上显示为绿色)
    void setRecommendedRange(double lo, double hi) {
        rec_lo_ = lo; rec_hi_ = hi; has_rec_ = true; update();
    }
    /// 设置临界范围 (色带上显示为红色, 超出即不建议)
    void setCriticalRange(double lo, double hi) {
        crit_lo_ = lo; crit_hi_ = hi; has_crit_ = true; update();
    }
    /// 设置安全范围 (规则十一): 超出 [lo, hi] 时边框变红 + 背景浅红,
    /// 用 dynamic property "hasWarning" 触发 QSS 规则, 不影响 focus 外圈动画.
    void setSafeRange(double lo, double hi);
    void clearSafeRange();

    // Focus 外圈动画 getter/setter
    double focusRingOpacity() const { return focus_ring_opacity_; }
    void setFocusRingOpacity(double v) { focus_ring_opacity_ = v; update(); }

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void focusInEvent(QFocusEvent*) override;
    void focusOutEvent(QFocusEvent*) override;

private:
    void checkSafeRange(double v);
    double rec_lo_ = 0, rec_hi_ = 0;
    double crit_lo_ = 0, crit_hi_ = 0;
    bool has_rec_ = false;
    bool has_crit_ = false;
    bool has_safe_ = false;
    double safe_lo_ = 0, safe_hi_ = 0;
    double focus_ring_opacity_ = 0.0;
    bool currently_unsafe_ = false;
};

/**
 * @brief 特斯拉风水平滑块
 * @details 继承 QSlider, 完全自绘 paintEvent, 不经过 QStyle.
 *   - 轨道 3px 高 #E5E7EB
 *   - 已填充部分纯 #007AFF 无渐变
 *   - 滑块白色圆 + 1.5px 蓝色细描边, 下方 12 alpha 微阴影
 *   - hover 时滑块微放大 (7→8 px), 靠 QPropertyAnimation 平滑过渡
 *   保留 QSlider 原生的 mouse/key/valueChanged 信号, 替换零改调用方.
 */
class SmoothSlider : public QSlider {
    Q_OBJECT
    Q_PROPERTY(double thumbScale READ thumbScale WRITE setThumbScale)
public:
    explicit SmoothSlider(Qt::Orientation orientation = Qt::Horizontal,
                          QWidget* parent = nullptr);
    double thumbScale() const { return thumb_scale_; }
    void setThumbScale(double s) { thumb_scale_ = s; update(); }

protected:
    void paintEvent(QPaintEvent*) override;
    void enterEvent(QEvent*) override;
    void leaveEvent(QEvent*) override;

private:
    double thumb_scale_ = 1.0;
};

/**
 * @brief iOS 标准拨动开关 (iOS 融合风)
 * @details 继承 QAbstractButton (checkable), 自绘 51×31 iOS 标准开关:
 *   - 关闭态: 轨道 #E5E5EA + 白滑块
 *   - 开启态: 轨道绿色 #34C759 + 白滑块 (iOS 开关是绿的, 不是蓝)
 *   - 切换用 OutBack 弹簧 350ms, overshoot=1.5 (有明显回弹)
 *   - 颜色用 InOutQuad 200ms 平滑过渡 (颜色永远不用弹簧)
 *   emit toggled(bool) — 与 QCheckBox 兼容, 可作为 drop-in 替代.
 */
class SmoothToggle : public QAbstractButton {
    Q_OBJECT
    Q_PROPERTY(double thumbX     READ thumbX     WRITE setThumbX)
    Q_PROPERTY(double thumbScale READ thumbScale WRITE setThumbScale)
    Q_PROPERTY(QColor trackColor READ trackColor WRITE setTrackColor)
public:
    explicit SmoothToggle(QWidget* parent = nullptr);
    double thumbX() const { return thumb_x_; }
    void setThumbX(double x) { thumb_x_ = x; update(); }
    double thumbScale() const { return thumb_scale_; }
    void setThumbScale(double s) { thumb_scale_ = s; update(); }
    QColor trackColor() const { return track_color_; }
    void setTrackColor(const QColor& c) { track_color_ = c; update(); }
    QSize sizeHint() const override { return QSize(51, 31); }  // iOS 标准

protected:
    void paintEvent(QPaintEvent*) override;
    void checkStateSet() override;       // setChecked(...) / 点击切换 统一入口
    void resizeEvent(QResizeEvent*) override;
    void enterEvent(QEvent*) override;
    void leaveEvent(QEvent*) override;

private:
    void animateToState();
    double thumb_x_      = 2.0;
    double thumb_scale_  = 1.0;   // hover 时 1.0 → 1.08
    QColor track_color_;
};

/**
 * @brief iOS 分段控制器 (Segmented Control, 融合风第八部分)
 * @details 横向分段选择, 白色滑块在分段间弹簧滑动. 典型用法:
 *   auto* seg = new SegmentedControl({"经济","标准","高功率"});
 *   connect(seg, &SegmentedControl::selectionChanged, ...);
 *   seg->setSelectedIndex(0);
 */
class SegmentedControl : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double thumbX     READ thumbX     WRITE setThumbX)
    Q_PROPERTY(double thumbWidth READ thumbWidth WRITE setThumbWidth)
public:
    explicit SegmentedControl(const QStringList& segments, QWidget* parent = nullptr);
    void setSelectedIndex(int index);
    int  selectedIndex() const { return selected_index_; }
    double thumbX() const { return thumb_x_; }
    void setThumbX(double x) { thumb_x_ = x; update(); }
    double thumbWidth() const { return thumb_width_; }
    void setThumbWidth(double w) { thumb_width_ = w; update(); }
signals:
    void selectionChanged(int index);
protected:
    void resizeEvent(QResizeEvent*) override;
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
private:
    QStringList segments_;
    int    selected_index_ = 0;
    double thumb_x_        = 2.0;
    double thumb_width_    = 0.0;
};

/**
 * @brief iOS 按压缩放按钮 (融合风第十部分)
 * @details 继承 QPushButton, 按下 scale(0.97) + 松开弹回 (OutBack).
 *   视觉完全自绘 (Squircle 圆角 + iOS 纯色底 + 白字), 不依赖 QSS.
 *   支持 4 种 Variant 颜色: Primary/Success/Danger/Warn — 均用 iOS 标准色板.
 *   用于"应用参数"/"开始执行"/"紧急停止"等关键操作按钮.
 */
class IOSButton : public QPushButton {
    Q_OBJECT
    Q_PROPERTY(double pressScale     READ pressScale     WRITE setPressScale)
    Q_PROPERTY(double hoverIntensity READ hoverIntensity WRITE setHoverIntensity)
public:
    enum Variant {
        Primary,   ///< iOS 蓝 #007AFF, 主操作
        Success,   ///< iOS 绿 #34C759, 正向 / "开始执行"
        Danger,    ///< iOS 红 #FF3B30, 危险 / "紧急停止"
        Warn,      ///< iOS 橙 #FF9500, 警告 / "暂停"
        Secondary, ///< 白底 + 浅灰边 + 深字, 次要/辅助操作
    };
    Q_ENUM(Variant)

    explicit IOSButton(const QString& text = QString(),
                       Variant variant = Primary,
                       QWidget* parent = nullptr);
    void setVariant(Variant v) { variant_ = v; update(); }
    Variant variant() const { return variant_; }

    double pressScale() const { return press_scale_; }
    void setPressScale(double s) { press_scale_ = s; update(); }
    double hoverIntensity() const { return hover_intensity_; }
    void setHoverIntensity(double v) { hover_intensity_ = v; update(); }
protected:
    void mousePressEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void enterEvent(QEvent*) override;
    void leaveEvent(QEvent*) override;
    void paintEvent(QPaintEvent*) override;
private:
    double  press_scale_     = 1.0;
    double  hover_intensity_ = 0.0;    // 0=离开态, 1=悬停态, 阴影参数按此混合
    Variant variant_         = Primary;
};

/**
 * @brief iOS 风成功 HUD (融合风补充五)
 * @details 弹出式反馈, 1.8 秒后自动消失. 静态用法:
 *   SuccessHUD::show(this, "参数已应用");
 * 视觉: 120×120 深色半透明圆角 + 白色勾号动画 + 文字
 */
class SuccessHUD : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double hudOpacity READ hudOpacity WRITE setHudOpacity)
    Q_PROPERTY(double hudScale   READ hudScale   WRITE setHudScale)
public:
    static void show(QWidget* parent, const QString& text = QStringLiteral("已应用"));
    double hudOpacity() const { return hud_opacity_; }
    void setHudOpacity(double v) { hud_opacity_ = v; update(); }
    double hudScale() const { return hud_scale_; }
    void setHudScale(double v) { hud_scale_ = v; update(); }
protected:
    void paintEvent(QPaintEvent*) override;
private:
    SuccessHUD(QWidget* parent, const QString& text);
    void animate();
    QString text_;
    double  hud_opacity_    = 0.0;
    double  hud_scale_      = 0.8;
    double  check_progress_ = 0.0;
    QTimer* check_timer_    = nullptr;
};

//==============================================================================
// 阶段三 · 布局系统 (融合风补充篇·补充一/二/十)
//==============================================================================

/**
 * @brief iOS 单行参数容器 (补充篇·补充一)
 * @details 固定 52 px 高, 左侧标签 + 右侧控件. hover 时整行背景平滑过渡到
 *   Theme::bgHover (#F3F3F5). 用于 GroupedSection 内部一行=一个参数.
 *
 *   构造时传入 (label, control), 可选挂尾部小字注释 / 单位.
 *   typedef:
 *     ParameterRow* row = new ParameterRow("斜移角度", spin, this);
 *     row->setUnit("°");          // 显示在控件右侧的单位
 *     row->setHint("0=水平");     // 显示在标签下方的灰色提示
 *
 * 视觉:
 *   - QHBoxLayout, contents margins (20, 0, 20, 0)
 *   - 高度由 cellHeight + 8 = 52 (Spacing 常量推导)
 *   - 自绘背景 bgColor (Q_PROPERTY, 由 animateColor 在 hover 时平滑)
 */
class ParameterRow : public QWidget {
    Q_OBJECT
    Q_PROPERTY(QColor bgColor READ bgColor WRITE setBgColor)
public:
    explicit ParameterRow(const QString& label,
                          QWidget* control,
                          QWidget* parent = nullptr);
    /// 用现有的 QWidget 作为标签 (例如 ParamLabel 这种带 hover 详情的标签).
    explicit ParameterRow(QWidget* labelWidget,
                          QWidget* control,
                          QWidget* parent = nullptr);

    /// 在控件右侧追加单位 (例如 "°" / "m" / "mm").
    void setUnit(const QString& unit);
    /// 在标签下方添加灰色提示文字.
    void setHint(const QString& hint);
    /// 把控件换成另一个 (布局上替换原 control_)
    void setControl(QWidget* control);

    QColor bgColor() const { return bg_color_; }
    void setBgColor(const QColor& c) { bg_color_ = c; update(); }

protected:
    void paintEvent(QPaintEvent*) override;
    void enterEvent(QEvent*) override;
    void leaveEvent(QEvent*) override;

private:
    QLabel* lbl_       = nullptr;
    QLabel* hint_lbl_  = nullptr;
    QLabel* unit_lbl_  = nullptr;
    QWidget* control_  = nullptr;
    class QHBoxLayout* row_layout_ = nullptr;
    class QVBoxLayout* label_col_  = nullptr;
    QColor bg_color_;
};

/**
 * @brief 滚动指示器 (补充篇·补充二)
 * @details 3px 宽的细条, 仅在滚动 1.5s 内可见, 之后淡出.
 *   配合 RubberScrollArea 使用, 不接收鼠标事件 (transparent).
 */
class ScrollIndicator : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double opacity READ opacity WRITE setOpacity)
public:
    explicit ScrollIndicator(QWidget* parent = nullptr);
    /// 通知有滚动发生, 显示 1.5s 后自动淡出
    void notifyScroll(int contentY, int contentHeight, int viewportHeight);
    double opacity() const { return opacity_; }
    void setOpacity(double v) { opacity_ = v; update(); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    double opacity_ = 0.0;
    QRect  bar_rect_;
    QTimer* fade_timer_ = nullptr;
};

/**
 * @brief 弹性滚动区 (补充篇·补充二)
 * @details 继承 QScrollArea, 默认隐藏标准滚动条, 用内嵌 ScrollIndicator
 *   显示进度. 滚到顶/底时滚轮带阻尼偏移 (resistance=3, 上限 80px),
 *   松手后弹回 (springAnimate, OutBack 350ms overshoot=1.2).
 *
 *   性能:
 *   - 滚动事件只 update(indicator_rect_), 不重绘整个内容.
 *   - 阻尼偏移用 setViewportMargins + scrollContentsBy 路径会引发整体重布局,
 *     这里用平移内容控件 (move) 实现, 视觉等价但成本低得多.
 */
class RubberScrollArea : public QScrollArea {
    Q_OBJECT
    Q_PROPERTY(int rubberOffset READ rubberOffset WRITE setRubberOffset)
public:
    explicit RubberScrollArea(QWidget* parent = nullptr);

    int rubberOffset() const { return rubber_offset_; }
    void setRubberOffset(int v);

protected:
    void wheelEvent(QWheelEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;
    void scrollContentsBy(int dx, int dy) override;

private:
    void springBack();
    void updateIndicator();

    int rubber_offset_ = 0;          ///< 当前橡皮筋位移 (上溢正, 下溢负)
    ScrollIndicator* indicator_ = nullptr;
    QPoint content_origin_;          ///< widget()->pos() 静止位置
    bool spring_running_ = false;
};

/**
 * @brief 底部固定操作栏 (补充篇·补充十)
 * @details 90 px 高, 顶部 12 px 渐隐遮罩 + 下方实色 Theme::bgWindow.
 *   内含一个 IOSButton, 点击转发为 applyClicked() 信号.
 */
class StickyFooter : public QWidget {
    Q_OBJECT
public:
    explicit StickyFooter(const QString& applyText,
                          QWidget* parent = nullptr);
    /// 暴露主按钮以便外部加文字提示 / 改 variant
    IOSButton* button() const { return apply_btn_; }

signals:
    void applyClicked();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    IOSButton* apply_btn_ = nullptr;
};

//==============================================================================
// 阶段四 · 动效打磨 (融合风补充篇·补充三/六/七/八)
//==============================================================================

/**
 * @brief 列表项依次入场动画 (补充篇·补充三)
 * @details 给一组控件做"渐入 + 下方滑入"分批动画, 每个控件之间错开 staggerDelay 毫秒.
 *   常用于页面首次加载或 Tab 切换后, 让 ParameterRow 像 iOS 列表那样依次出现.
 *
 *   静态接口, 不持有状态:
 *     StaggerAnimator::animateIn({row1, row2, row3});         // 默认 50/400 ms
 *     StaggerAnimator::animateOut({row1, row2, row3});        // 默认 30/250 ms
 *
 *   实现细节:
 *     - 透明度: 走 QGraphicsOpacityEffect (按需创建并复用), 不污染父布局.
 *     - 位移:   动画 widget->pos() 从原位 + 20px 滑回原位; 仅适合页面加载场景,
 *               动画期间不要触发父级 re-layout, 否则 endValue 会被覆盖.
 *     - 用 QPointer 保护异步回调, 避免控件提前销毁导致的悬空指针.
 */
class StaggerAnimator : public QObject {
    Q_OBJECT
public:
    /// 渐入 + 下滑回原位 (页面进入)
    static void animateIn(const QList<QWidget*>& widgets,
                          int staggerDelay = 50,
                          int duration     = 400);
    /// 渐出 + 上移消失 (页面退出)
    static void animateOut(const QList<QWidget*>& widgets,
                           int staggerDelay = 30,
                           int duration     = 250);
};

/**
 * @brief iOS 圆形进度指示器 (补充篇·补充六)
 * @details 12 根渐隐线段绕中心旋转, 每秒 12 步 (~83 ms 一步), 视觉与 iOS 系统转圈一致.
 *   不接收鼠标事件, 内部用 QTimer 驱动重绘. 调用 start() 才显示 + 转动,
 *   stop() 立即隐藏 + 停转. 重复调用幂等.
 *
 *   典型用法 (覆盖在按钮上做"应用中"反馈):
 *     auto* spinner = new ActivityIndicator(applyBtn, 16);
 *     spinner->start();
 *     // ... 网络/CAN 操作完成后
 *     spinner->stop();
 */
class ActivityIndicator : public QWidget {
    Q_OBJECT
public:
    explicit ActivityIndicator(QWidget* parent = nullptr, int size = 20);
    void start();
    void stop();
    bool isRunning() const;

protected:
    void paintEvent(QPaintEvent*) override;

private:
    int     angle_ = 0;
    QTimer* timer_ = nullptr;
};

/**
 * @brief 骨架屏加载占位 (补充篇·补充七)
 * @details 灰色圆角块 + 持续从左到右扫过的高光, 用于参数从设备读取时的等待占位.
 *   高度由构造参数固定, 宽度由父布局或显式 setFixedWidth 决定.
 *   动画在构造时自动启动, 控件销毁时一并停止 (动画 parent = this).
 */
class SkeletonBlock : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double shimmerPosition READ shimmerPosition WRITE setShimmerPosition)
public:
    explicit SkeletonBlock(QWidget* parent = nullptr, int height = 16);
    double shimmerPosition() const { return shimmer_pos_; }
    void   setShimmerPosition(double v) { shimmer_pos_ = v; update(); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    double shimmer_pos_ = -1.0;
};

/**
 * @brief iOS 风长按上下文菜单 (补充篇·补充八)
 * @details 在指定坐标弹出半透明圆角菜单, 项目数任意, 危险操作可指定红色文字.
 *   失焦时自动渐出销毁; 子项点击后触发 action 并销毁.
 *
 *   示例:
 *     IOSContextMenu::show(this, mapToGlobal(e->pos()), {
 *         {"重置默认值", QColor(),                  [this](){ resetToDefault(); }},
 *         {"复制当前值", QColor(),                  [this](){ copyValue(); }},
 *         {"删除自定义", QColor(255, 59, 48),       [this](){ deleteCustom(); }},
 *     });
 *
 *   实现说明:
 *     - 顶层 Qt::Popup 窗口 + WA_TranslucentBackground, 失焦自动关闭, 不污染父级.
 *     - 阴影走 ShadowUtils::drawPopoverShadow, 形状走 Squircle (圆角 14).
 *     - hover 高亮按 m_hoverIndex 重绘, 不创建 hover 子控件.
 */
class IOSContextMenu : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double menuOpacity READ menuOpacity WRITE setMenuOpacity)
    Q_PROPERTY(double menuScale   READ menuScale   WRITE setMenuScale)
public:
    struct MenuItem {
        QString                 title;
        QColor                  textColor;   ///< 无效则回退 Theme::textPrimary
        std::function<void()>   action;
    };

    /// 全局坐标 globalPos 处弹出菜单, 自动定位 + 防越界
    static void show(QWidget* anchor,
                     const QPoint& globalPos,
                     const QList<MenuItem>& items);

    double menuOpacity() const { return menu_opacity_; }
    void   setMenuOpacity(double v) { menu_opacity_ = v; update(); }
    double menuScale() const { return menu_scale_; }
    void   setMenuScale(double v) { menu_scale_ = v; update(); }

protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void focusOutEvent(QFocusEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private:
    IOSContextMenu(const QPoint& globalPos, const QList<MenuItem>& items);
    void animateIn();
    void animateOut();

    QList<MenuItem> items_;
    double          menu_opacity_ = 0.0;
    double          menu_scale_   = 0.9;
    int             hover_index_  = -1;
    bool            closing_      = false;
    static constexpr int kItemH    = 44;
    static constexpr int kMenuW    = 220;
    static constexpr int kPadding  = 24;   ///< 留给阴影绘制的画布外延
};

class ManualControlWidget : public QWidget {
    Q_OBJECT

public:
    explicit ManualControlWidget(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~ManualControlWidget() override;

    // 自动整定数据类型 (public, 供 AutoTuneCurveWidget 访问)
    struct AutoTuneSample {
        double time_sec;    ///< 相对时间(秒)
        double target;      ///< 目标值
        double actual;      ///< 实际值
        double control_out; ///< 控制输出
    };

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    void onDirectTimerTick();    ///< 50Hz  直接CAN
    void onNodeTimerTick();      ///< 50Hz  节点模式
    void onStatusTimerTick();    ///< 5Hz   状态刷新 + 通信诊断

private:
    //==========================================================================
    // 初始化
    //==========================================================================
    void setupUI();
    QWidget* createSidebar();
    void emergencyStop();  ///< 全局急停
    void setupTab1_DirectCAN(QTabWidget* tabs);
    void setupTab2_NodeTest(QTabWidget* tabs);
    void setupTab3_PIDTuning(QTabWidget* tabs);
    void setupTab3_AutoTune(QVBoxLayout* layout);  ///< 铲刀PID自动整定UI
    void setupTab4_VehicleStatus(QTabWidget* tabs);
    void setupTab5_PlanningParams(QTabWidget* tabs);
    void setupTab6_VehicleConfig(QTabWidget* tabs);
    void setupTab7_DataRecord(QTabWidget* tabs);
    void setupTab8_CommDiag(QTabWidget* tabs);
    void setupTab9_DiagSnapshot(QTabWidget* tabs);

    void setupPublishers();
    void setupSubscribers();
    void setupConnections();

    QPushButton* createHoldButton(const QString& text, const QString& style = "");
    QDoubleSpinBox* makeSpin(double min, double max, double val, double step, int dec);
    QGroupBox* makeGroup(const QString& title);
    /// 创建可点击参数标签（悬停提示 + 点击弹窗详解）
    ParamLabel* makeParamLabel(const QString& text,
                               const QString& tooltip,
                               const QString& detail);

    // 安全: Tab切换时暂停/恢复定时器
    void onTabChanged(int index);

    // 持久化: XML保存/加载所有参数
    void saveConfig(const QString& path);
    void loadConfig(const QString& path);

    // 指令发布
    void publishLever1();
    void publishMoldboard();
    void publishNodeCommands();

    //==========================================================================
    // ROS 回调
    //==========================================================================
    void llaCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void angleCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void walkStateCallback(const std_msgs::Float64::ConstPtr& msg);
    void decisionStatusCallback(const std_msgs::Float64::ConstPtr& msg);
    void moldboardActualCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void vRightCallback(const std_msgs::Float64::ConstPtr& msg);
    void vLeftCallback(const std_msgs::Float64::ConstPtr& msg);
    void terminalFlagCallback(const std_msgs::Float64::ConstPtr& msg);
    void moldboardDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // 整车状态
    void engineSpeedCallback(const std_msgs::Float64::ConstPtr& msg);
    void engineTorqueCallback(const std_msgs::Int8::ConstPtr& msg);
    void transSpeedCallback(const std_msgs::Int16::ConstPtr& msg);
    void vehicleSpeedCallback(const std_msgs::Float64::ConstPtr& msg);
    void outputCurrentCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
    void imuPitchCallback(const std_msgs::Float64::ConstPtr& msg);
    void imuRollCallback(const std_msgs::Float64::ConstPtr& msg);

    // 通信诊断: 记录话题最后更新时间
    void recordTopicTime(const QString& topic);

    //==========================================================================
    // ROS
    //==========================================================================
    ros::NodeHandle& nh_;
    QTabWidget* tabs_;  ///< 主Tab控件 (用于Tab切换信号)
    // 底部抽屉
    QWidget* drawer_handle_;          ///< 拖拽手柄
    bool drawer_open_ = false;        ///< 抽屉是否展开
    int drawer_height_ = 500;         ///< 抽屉展开高度 (默认较大)
    QPropertyAnimation* drawer_anim_; ///< 抽屉动画
    QShortcut* shortcut_toggle_tab_;  ///< F11 快捷键
    bool drawer_dragging_ = false;    ///< 是否正在拖拽手柄
    int drawer_drag_start_y_ = 0;     ///< 拖拽起始鼠标Y
    int drawer_drag_start_h_ = 0;    ///< 拖拽起始高度
    void toggleDrawer();              ///< 切换抽屉展开/收起
    void onTabBarClicked(int index);  ///< Tab标签点击处理
    QTimer* direct_timer_;
    QTimer* node_timer_;
    QTimer* status_timer_;

    // 直接CAN
    ros::Publisher pub_lever1_, pub_lever_moldboard_, pub_moldboard_flag_direct_;
    ros::Publisher pub_engine_speed_, pub_brake_;
    ros::Publisher pub_mode_switch_;              ///< /Mode_Switch → ros_to_can → 有人/无人CAN帧
    // 节点控制
    ros::Publisher pub_main_switch_, pub_main_switch_ctrl_, pub_detection_;
    ros::Publisher pub_walk_state_, pub_terminal_, pub_reference_;
    ros::Publisher pub_mold_ctrl_flag_, pub_ref_height_, pub_ref_angle_;
    ros::Publisher pub_set_blade_origin_;     ///< [Issue#7] /decision/set_blade_origin
    // PID参数
    ros::Publisher pub_walk_params_, pub_mold_params_;
    // 规划参数
    ros::Publisher pub_test_config_;  ///< 测试模式参数
    ros::Publisher pub_test_script_; ///< 测试脚本
    ros::Publisher pub_full_path_preview_; ///< 脚本路径预览 (发到/decision/full_path)
    ros::Publisher pub_regen_path_;  ///< 手动触发路径生成
    ros::Publisher pub_path_mode_;   ///< 路径工艺模式 (0=顶端斜移, 1=底端前进斜移, 2=底端倒车斜移)
    ros::Publisher pub_path_params_; ///< 运行时路径参数
    ros::Publisher pub_path_stop_; ///< 紧急停止

    // 订阅
    ros::Subscriber sub_lla_, sub_angle_, sub_imu_, sub_walk_state_;
    ros::Subscriber sub_decision_status_, sub_moldboard_actual_;
    ros::Subscriber sub_v_right_, sub_v_left_, sub_terminal_flag_, sub_moldboard_debug_;
    ros::Subscriber sub_engine_speed_, sub_engine_torque_, sub_trans_speed_;
    ros::Subscriber sub_vehicle_speed_, sub_output_current_;
    ros::Subscriber sub_imu_pitch_, sub_imu_roll_;
    // [Fix-v19] 订阅 moldboard_controller 发布的铲刀 IMU 横滚角
    //   autotune 角度通道用此变量替代 /IMU_Roll (旧款旧 IMU, 硬件不存在)
    ros::Subscriber sub_mold_imu_roll_;
    // 新增订阅
    ros::Subscriber sub_coolant_temp_, sub_engine_hours_;
    ros::Subscriber sub_gear_pos_, sub_gear_dir_;
    ros::Subscriber sub_hand_brake_, sub_hydraulic_lock_, sub_brake_valve_;
    ros::Subscriber sub_fuel_level_, sub_fuel_total_;
    ros::Subscriber sub_manual_auto_, sub_handle_turn_;
    ros::Subscriber sub_vcu_error_, sub_sys_error_;
    ros::Subscriber sub_mil_light_, sub_red_stop_;
    ros::Subscriber sub_map_info_;  ///< 栅格地图尺寸
    ros::Subscriber sub_exec_state_;    ///< 执行器状态
    ros::Subscriber sub_wp_index_;      ///< 当前路径点索引

    //==========================================================================
    // 左侧边栏
    //==========================================================================
    QLabel* side_rtk_status_;
    QLabel* side_main_switch_;
    QLabel* side_state_machine_;
    QLabel* side_walk_state_;
    QLabel* side_blade_height_;
    QLabel* side_engine_rpm_;
    QLabel* side_vehicle_speed_;
    QLabel* side_heading_;
    QLabel* side_can_status_;
    QLabel* side_config_path_;       ///< 侧边栏: 当前配置文件名
    // 感知风险状态灯 (前/后/左/右)
    QLabel* side_risk_front_;
    QLabel* side_risk_back_;
    QLabel* side_risk_left_;
    QLabel* side_risk_right_;
    IOSButton*   btn_emergency_stop_;       ///< 侧边栏急停/恢复开关 (Danger/Success 互切)
    bool emergency_stopped_ = false;

    //==========================================================================
    // Tab1 直接CAN
    //==========================================================================
    QPushButton* btn_forward_, *btn_backward_, *btn_left_, *btn_right_;
    SmoothSlider* slider_steering_;  QLabel* label_steering_val_;
    QPushButton* btn_blade_up_, *btn_blade_down_, *btn_blade_tilt_left_, *btn_blade_tilt_right_;
    SmoothSlider* slider_blade_speed_;  QLabel* label_blade_speed_val_;
    QCheckBox* chk_blade_enable_;
    QPushButton* btn_unlock_, *btn_stop_, *btn_gear_up_, *btn_gear_down_;
    SmoothToggle* btn_unmanned_mode_;            ///< iOS 绿色拨动: 有人/无人模式
    QPushButton* btn_brake_;                     ///< 刹车按钮 (按住发1, 松开发0)
    bool brake_pressed_ = false;                 ///< 刹车按住标志
    QLabel* label_gear_dir_display_;             ///< Tab1 挡位方向 (N/F/R)
    QLabel* label_gear_display_;                 ///< Tab1 挡位大小 (1~5)

    //==========================================================================
    // Tab2 节点测试
    //==========================================================================
    enum NodeTestMode { MODE_AUTO=0, MODE_WALK=1, MODE_BLADE=2 };
    SegmentedControl* mode_group_;          ///< 自动/行走/铲刀 3 选 1 iOS 分段
    QGroupBox* group_auto_, *group_walk_, *group_blade_;
    SmoothToggle* btn_node_main_switch_;    ///< iOS 绿色拨动: 主开关
    SmoothToggle* btn_node_detection_;      ///< iOS 绿色拨动: 感知就绪
    IOSButton*   btn_node_drive_;     ///< Primary
    IOSButton*   btn_node_rotate_;    ///< Primary
    IOSButton*   btn_node_stop_;      ///< Danger
    QDoubleSpinBox* spin_x_terminal_, *spin_theta_terminal_, *spin_v_ref_, *spin_omega_ref_;
    QCheckBox* chk_node_mold_enable_;
    QDoubleSpinBox* spin_ref_height_, *spin_ref_angle_;
    // [Issue#7] 铲刀高度基准点 UI
    QGroupBox* group_blade_origin_;
    QDoubleSpinBox* spin_blade_origin_lat_, *spin_blade_origin_lon_, *spin_blade_origin_alt_;
    IOSButton*   btn_lock_blade_origin_;     ///< Primary 锁定 RTK 位置
    QLabel* label_blade_origin_status_;
    QLabel* label_v_right_, *label_v_left_, *label_terminal_flag_, *label_mold_debug_;
    QTextEdit* log_output_;
    GridMapWidget* map_widget_;

    // 主显示区切换: 联合施工动画 ↔ 栅格地图 ↔ RViz 感知视图
    QStackedWidget* main_display_stack_ = nullptr;
    RvizPerceptionWidget* rviz_widget_ = nullptr;
    QLabel* animation_widget_ = nullptr;    ///< 联合施工动画页 (QMovie)
    class QMovie* animation_movie_ = nullptr;
    QPushButton* btn_view_animation_ = nullptr;
    QPushButton* btn_view_gridmap_ = nullptr;
    QPushButton* btn_view_rviz_ = nullptr;

    //==========================================================================
    // Tab3 PID调参
    //==========================================================================
    QDoubleSpinBox* spin_kp_x_, *spin_ki_x_, *spin_kd_x_;
    QDoubleSpinBox* spin_kp_theta_, *spin_ki_theta_, *spin_kd_theta_;
    QDoubleSpinBox* spin_x_tol_, *spin_theta_tol_, *spin_track_width_;
    QDoubleSpinBox* spin_td_r_, *spin_td_h_;
    QDoubleSpinBox* spin_gear_dz_, *spin_steer_dz_;  ///< 桥接死区参数
    QDoubleSpinBox* spin_kp_h_up_, *spin_kp_h_dn_, *spin_ki_h_up_, *spin_ki_h_dn_;
    QDoubleSpinBox* spin_dz_height_, *spin_imax_height_;
    QDoubleSpinBox* spin_kp_t_up_, *spin_kp_t_dn_, *spin_ki_t_up_, *spin_ki_t_dn_;
    QDoubleSpinBox* spin_dz_theta_, *spin_imax_theta_;
    IOSButton*   btn_send_walk_params_;   ///< Success 发送行走参数
    IOSButton*   btn_send_mold_params_;   ///< Success 发送铲刀参数
    QLabel* label_walk_params_ack_;   ///< 行走参数发送反馈
    QLabel* label_mold_params_ack_;   ///< 铲刀参数发送反馈

    //==========================================================================
    // Tab4 整车状态 — 仪表盘风格
    //==========================================================================
    // 核心圆形仪表
    CircularGauge* gauge_engine_rpm_;       ///< 发动机转速 (rpm)
    CircularGauge* gauge_vehicle_speed_;    ///< 车速 (m/s)
    CircularGauge* gauge_coolant_temp_;     ///< 冷却液温度 (°C)
    CircularGauge* gauge_fuel_level_;       ///< 燃油量 (%)
    // 姿态水平仪
    LevelBar* bar_pitch_;                   ///< 俯仰角
    LevelBar* bar_roll_;                    ///< 横滚角
    // 状态胶囊 (开/关型状态)
    StatusPill* pill_hand_brake_;           ///< 手刹
    StatusPill* pill_hydraulic_lock_;       ///< 液压锁
    StatusPill* pill_brake_valve_;          ///< 制动阀
    StatusPill* pill_manual_auto_;          ///< 手动/自动
    // 辅助信息 (仍用文字显示的次要指标)
    QLabel* label_engine_torque_;           ///< 扭矩 %
    QLabel* label_engine_hours_;            ///< 工时
    QLabel* label_trans_speed_;             ///< 变速箱转速
    QLabel* label_gear_pos_, *label_gear_dir_;
    QLabel* label_lla_detail_, *label_heading_detail_;
    QLabel* label_fuel_total_;              ///< 总油耗
    QLabel* label_handle_turn_;             ///< 转向值
    QLabel* label_output_curr_;             ///< 输出电流 4 路
    QLabel* label_fault_summary_;           ///< 故障汇总 (顶部横幅)
    // Tab8 硬件状态
    StatusPill* label_hw_rtk_;      ///< Tab8 RTK 硬件在线状态 (iOS 胶囊)
    StatusPill* label_hw_imu_;      ///< Tab8 IMU 硬件在线状态
    StatusPill* label_hw_engine_;   ///< Tab8 ECU 硬件在线状态
    StatusPill* label_hw_can_;      ///< Tab8 CAN 硬件在线状态

    //==========================================================================
    // Tab5 规划参数
    //==========================================================================
    // 路径参数
    QDoubleSpinBox* spin_push_length_;       ///< 推土长度(m)
    QDoubleSpinBox* spin_blade_width_;       ///< 列宽/铲刀宽度(m) (模式1专用)
    QLabel* label_blade_width_tag_;          ///< "列宽(m):" 标签 (模式1显示)
    QDoubleSpinBox* spin_shift_angle_;       ///< 斜移角度(度)
    QDoubleSpinBox* spin_push_heading_;      ///< 推土方向角度(度)
    QDoubleSpinBox* spin_v_push_;            ///< 推土速度(m/s)
    QDoubleSpinBox* spin_v_reverse_;         ///< 倒车速度(m/s)
    QDoubleSpinBox* spin_omega_rotate_;      ///< 旋转角速度(deg/s)
    QDoubleSpinBox* spin_x_back_set_;        ///< 过载后退距离(m)
    QDoubleSpinBox* spin_path_pos_tol_;           ///< 到位距离容差(m)
    QDoubleSpinBox* spin_path_theta_tol_;         ///< 旋转到位容差(度)
    QLabel* label_calc_width_;               ///< 自动计算的列宽/斜移距离
    QLabel* label_map_info_;                 ///< 地图尺寸显示
    // 模式选择
    SmoothToggle* chk_test_mode_;   ///< "无地图模式" — iOS 51×31 绿色拨动开关
    SegmentedControl* combo_path_mode_;      ///< iOS 3 段: 换列模式 (底端前进/顶端/底端倒车)
    QWidget* test_panel_;
    QWidget* script_panel_;
    QWidget* map_panel_;                     ///< 有地图面板
    QDoubleSpinBox* spin_test_columns_;      ///< 无地图: 列数
    QDoubleSpinBox* spin_test_passes_;       ///< 无地图: 每列遍数
    QDoubleSpinBox* spin_map_scale_;         ///< 有地图: 缩放系数
    SegmentedControl* combo_map_start_corner_;      ///< 有地图: 起点位置
    SegmentedControl* spin_map_push_heading_; ///< iOS 4 段: 推土方向 (北/东/南/西)
    QTextEdit* edit_test_script_;            ///< 简单脚本
    IOSButton*   btn_run_script_;         ///< Success 执行脚本
    IOSButton*   btn_stop_script_;        ///< Danger 停止脚本
    QLabel* label_script_status_;
    // [3D-Level] 铲刀找平参数 (绑定到路径规划)
    SegmentedControl* combo_level_mode_;            ///< 找平模式: FLAT_3D(找平+横坡) / SLOPE_3D(纵坡+横坡)
    QDoubleSpinBox* spin_target_level_height_;  ///< 找平目标高度 (米, ENU-Up 绝对高程)
    QDoubleSpinBox* spin_blade_angle_deg_;   ///< 横坡角度 (度, 两个模式共用)
    QWidget* flat3d_panel_;                  ///< 找平+横坡 参数面板
    // [SLOPE_3D] 纵坡+横坡 参数面板
    QWidget* slope3d_panel_;                 ///< 纵坡+横坡 参数面板
    QDoubleSpinBox* spin_slope_start_height_;   ///< 纵坡起始高度 (米, ENU-Up)
    QDoubleSpinBox* spin_slope_gradient_;       ///< 纵向坡度 (%, 正上坡负下坡)
    // 辅助模块参数 (保留)
    QDoubleSpinBox* spin_tol_run_x_, *spin_tol_run_theta_, *spin_tol_mold_height_;
    QDoubleSpinBox* spin_speed_gain_risk_, *spin_speed_gain_mold_, *spin_mold_limit_;
    QDoubleSpinBox* spin_overload_trans_, *spin_overload_vehicle_, *spin_overload_angular_;
    // 操作按钮
    IOSButton*   btn_send_plan_params_;      ///< iOS 按压缩放 + Squircle 圆角
    IOSButton*   btn_generate_path_;         ///< 手动触发路径生成 (iOS 按压风)
    IOSButton*   btn_start_exec_;            ///< 开始执行 (iOS Success 绿)
    IOSButton*   btn_pause_exec_;            ///< 暂停执行 (iOS Warn 橙)
    IOSButton*   btn_path_stop_;             ///< 紧急停止 (iOS Danger 红)
    QLabel* label_plan_params_ack_;          ///< 参数发送反馈
    QLabel* label_exec_state_;               ///< 执行器状态
    QLabel* label_wp_progress_;              ///< 路径进度

    //==========================================================================
    // Tab6 车辆配置
    //==========================================================================
    QComboBox* combo_vehicle_preset_;
    IOSButton*   btn_save_config_;          ///< Primary
    IOSButton*   btn_load_config_;          ///< Success
    IOSButton*   btn_load_and_send_;        ///< Warn
    QLineEdit* edit_config_path_;
    // 参数管理增强
    IOSButton*   btn_save_preset_;       ///< Primary: 另存为预设
    IOSButton*   btn_delete_preset_;     ///< Danger: 删除预设
    QComboBox* combo_history_;           ///< 历史记录列表
    IOSButton*   btn_restore_history_;   ///< Success: 恢复历史
    IOSButton*   btn_view_changelog_;    ///< Secondary: 变更日志
    QMap<QString, double> last_saved_params_; ///< 上次保存的参数值 (用于diff)
    void refreshPresetList();            ///< 刷新预设列表
    void refreshHistoryList();           ///< 刷新历史列表
    void saveHistory(const QString& configPath); ///< 保存历史+变更日志
    QString getParamsDir() const;        ///< 获取参数目录

    //==========================================================================
    // Tab7 数据录制
    //==========================================================================
    IOSButton*   btn_start_record_;         ///< Tab7 rosbag 开始 (Success)
    IOSButton*   btn_stop_record_;          ///< Tab7 rosbag 停止 (Danger)
    QLabel* label_record_status_;
    QListWidget* record_topic_list_;    ///< 话题勾选列表
    QLineEdit* edit_custom_topic_;     ///< 自定义话题输入
    QLineEdit* edit_record_path_;      ///< 录制保存路径
    QProcess* rosbag_process_;
    QElapsedTimer record_elapsed_;
    bool recording_ = false;

    //==========================================================================
    // Tab8 通信诊断
    //==========================================================================
    struct TopicDiag {
        QLabel* name_label;
        QLabel* freq_label;
        QLabel* status_label;
        QLabel* value_label;
        int count = 0;
        qint64 last_time_ms = 0;
        QString last_value;       ///< 最新值文本
    };
    QMap<QString, TopicDiag> topic_diags_;
    QElapsedTimer diag_elapsed_;
    QVBoxLayout* diag_layout_;

    void addDiagTopic(const QString& name);
    /// 记录话题活跃时间 + 最新值
    void recordTopicValue(const QString& topic, const QString& value);

    //==========================================================================
    // 内部状态
    //==========================================================================
    bool forward_pressed_=false, backward_pressed_=false;
    bool left_pressed_=false, right_pressed_=false;
    bool unlock_pressed_=false, gear_up_pressed_=false, gear_down_pressed_=false;
    bool locked_=true;
    bool blade_up_pressed_=false, blade_down_pressed_=false;
    bool blade_tilt_l_pressed_=false, blade_tilt_r_pressed_=false;

    NodeTestMode current_mode_ = MODE_AUTO;
    bool node_main_switch_on_=false, node_detection_done_=false;
    int node_walk_mode_=0;

    // 缓存
    double v_right_val_=0, v_left_val_=0, terminal_flag_val_=0;
    double mold_u_height_=0, mold_u_theta_=0, mold_e_height_=0, mold_e_theta_=0;
    double engine_rpm_=0, engine_torque_=0, trans_speed_=0, vehicle_speed_=0;
    double imu_pitch_deg_=0, imu_roll_deg_=0;
    // [Fix-v19] 铲刀 IMU 横滚角 (订阅 /moldboard/imu_roll, 由 moldboard_controller 发布)
    //   autotune 角度通道和 GUI Roll 显示都改用这个, 替代不存在的 /IMU_Roll (旧款旧 IMU)
    double mold_imu_roll_deg_=0;
    uint16_t output_curr_[4] = {};
    double lla_lat_=0, lla_lon_=0, lla_alt_=0, heading_deg_=0;
    double walk_state_val_=0, decision_status_val_=0;
    int16_t mold_right_mm_=0, mold_left_mm_=0;
    // 新增缓存
    double coolant_temp_=0, engine_hours_=0;
    uint8_t gear_pos_=0, gear_dir_=0;
    uint8_t hand_brake_=0, hydraulic_lock_=0, brake_valve_=0;
    uint8_t fuel_level_=0;
    double fuel_total_=0;
    uint8_t manual_auto_=0, handle_turn_val_=0;
    uint32_t sys_error_code_=0;
    uint8_t mil_light_=0, red_stop_=0;
    int fault_count_=0;  ///< 活跃故障数
    uint32_t map_rows_=0, map_cols_=0;
    float map_res_=0;
    int8_t risk_state_[4] = {-1,-1,-1,-1}; ///< 感知风险状态 [前,后,左,右] -1=无数据 0=安全 1=危险 2=停车
    qint64 risk_state_last_ms_ = 0;         ///< 最后收到risk_state的时间

    //==========================================================================
    // Tab9 诊断快照
    //==========================================================================
    struct DiagSample {
        double timestamp;
        double enu_x, enu_y, heading;
        int exec_state, waypoint_index;
        double target_dist, heading_error;
        double walk_state, decision_status;
        double v_right, v_left, terminal_flag;
        double mold_height_l, mold_height_r;
        double vehicle_speed, engine_rpm;
        int gear_pos, gear_dir;
    };

    struct DiagEvent {
        double timestamp;
        QString type;
        QString description;
    };

    QLineEdit* edit_diag_path_;              ///< 保存路径
    QDoubleSpinBox* spin_diag_duration_;     ///< 录制时长(秒)
    QLineEdit* edit_diag_desc_;              ///< 问题描述
    IOSButton*   btn_diag_save_;             ///< Tab9 开始录制 (Warn)
    IOSButton*   btn_diag_cancel_;           ///< Tab9 取消录制 (Danger)
    QProgressBar* diag_progress_;            ///< 录制进度条
    QLabel* label_diag_status_;              ///< 录制状态
    QLabel* label_diag_last_save_;           ///< 上次保存信息
    QTextEdit* text_diag_events_;            ///< 操作事件日志显示

    QTimer* diag_timer_;                     ///< 采样定时器 (仅录制时运行)
    std::deque<DiagSample> diag_samples_;    ///< 录制数据
    std::deque<DiagEvent> diag_events_;      ///< 操作事件缓冲
    int diag_sample_rate_ = 10;              ///< 采样频率(Hz)
    bool diag_recording_ = false;            ///< 是否正在录制
    int diag_target_samples_ = 0;            ///< 目标采样数
    int diag_recorded_count_ = 0;            ///< 已采样数
    double diag_record_start_time_ = 0;      ///< 录制开始时间戳

    // 诊断数据缓存
    int diag_exec_state_ = 0;
    int diag_wp_index_ = 0;
    double diag_target_dist_ = 0;
    double diag_heading_error_ = 0;
    double diag_enu_x_ = 0, diag_enu_y_ = 0;

    // 诊断订阅
    ros::Subscriber sub_diag_exec_state_;
    ros::Subscriber sub_diag_wp_index_;
    ros::Subscriber sub_diag_error_;
    ros::Subscriber sub_diag_enu_;

    /// 记录操作事件 (供各Tab按钮调用)
    void recordDiagEvent(const QString& type, const QString& desc);
    /// 录制完成, 写入CSV
    void diagStopAndSave();

    //==========================================================================
    // 铲刀PID自动整定
    //==========================================================================
    enum AutoTunePhase {
        AT_IDLE = 0,        ///< 空闲
        AT_STEP_UP,         ///< 正阶跃中 (+幅度)
        AT_WAIT_UP,         ///< 等待正阶跃稳定
        AT_STEP_DN,         ///< 负阶跃中 (-幅度)
        AT_WAIT_DN,         ///< 等待负阶跃稳定
        AT_ANALYZE,         ///< 分析结果
        AT_WAIT_USER,       ///< 等待用户确认
    };
    enum AutoTuneChannel {
        AT_CH_HEIGHT = 0,   ///< 高度通路
        AT_CH_THETA  = 1,   ///< 角度通路
    };

    struct StepAnalysis {
        double rise_time    = 0;   ///< 上升时间(秒)
        double overshoot    = 0;   ///< 超调量(%)
        double settle_time  = 0;   ///< 调节时间(秒)
        double steady_error = 0;   ///< 稳态误差
        bool   oscillating  = false; ///< 是否震荡
    };

    // UI
    QDoubleSpinBox* spin_at_step_mm_;     ///< 阶跃幅度(mm)
    QSpinBox* spin_at_max_iter_;          ///< 最大迭代次数
    QDoubleSpinBox* spin_at_settle_sec_;  ///< 稳定等待时间(秒)
    SegmentedControl* combo_at_channel_;  ///< iOS 2 段: 高度/角度
    IOSButton*   btn_at_start_;           ///< 开始标定
    IOSButton*   btn_at_apply_;           ///< 应用并继续
    IOSButton*   btn_at_auto_;            ///< 自动继续
    IOSButton*   btn_at_stop_;            ///< 停止
    QLabel* label_at_status_;             ///< 状态
    QLabel* label_at_iter_;               ///< 迭代次数
    QLabel* label_at_analysis_;           ///< 分析结果文本
    QWidget* at_curve_widget_;            ///< 响应曲线绘图区
    QLabel* label_at_suggest_;            ///< 建议参数

    // 状态
    AutoTunePhase at_phase_ = AT_IDLE;
    AutoTuneChannel at_channel_ = AT_CH_HEIGHT;
    int at_iteration_ = 0;
    bool at_auto_mode_ = false;          ///< 自动继续模式
    double at_baseline_ = 0;             ///< 阶跃前基准值
    double at_target_val_ = 0;           ///< 阶跃目标值
    double at_step_start_time_ = 0;      ///< 阶跃开始时间
    double at_settle_deadline_ = 0;      ///< 稳定截止时间
    QTimer* at_timer_ = nullptr;         ///< 采样定时器 (50Hz)
    std::vector<AutoTuneSample> at_samples_up_;   ///< 正阶跃数据
    std::vector<AutoTuneSample> at_samples_dn_;   ///< 负阶跃数据
    StepAnalysis at_result_up_;          ///< 正阶跃分析
    StepAnalysis at_result_dn_;          ///< 负阶跃分析

    // 在发阶跃前保存的原始ref值
    double at_original_ref_height_ = 0;
    double at_original_ref_angle_  = 0;

    void atStartStep(bool up);           ///< 开始一次阶跃
    void atSampleTick();                 ///< 采样定时器回调
    StepAnalysis atAnalyzeStep(const std::vector<AutoTuneSample>& samples, double step_mm);
    void atShowResults();                ///< 显示分析结果和曲线
    void atApplyNewParams();             ///< 根据分析结果计算并应用新PI参数
    void atStop();                       ///< 停止整定,恢复原始ref

    /// 根据分析结果计算新PI参数 (公共逻辑, showResults 和 applyNewParams 共用)
    struct PIParams { double kp_up, kp_dn, ki_up, ki_dn; };
    PIParams atCalcNewParams();

    //==========================================================================
    // 行走PID自动整定
    //==========================================================================
    enum WalkTuneChannel {
        WT_CH_DISTANCE = 0,  ///< 距离通路
        WT_CH_HEADING  = 1,  ///< 航向通路
    };

    // UI
    QDoubleSpinBox* spin_wt_step_;        ///< 阶跃幅度 (m 或 deg)
    QSpinBox* spin_wt_max_iter_;          ///< 最大迭代次数
    QDoubleSpinBox* spin_wt_settle_sec_;  ///< 稳定等待时间(秒)
    SegmentedControl* combo_wt_channel_;  ///< iOS 2 段: 距离/航向
    IOSButton*   btn_wt_start_;
    IOSButton*   btn_wt_apply_;
    IOSButton*   btn_wt_auto_;
    IOSButton*   btn_wt_stop_;
    QLabel* label_wt_status_;
    QLabel* label_wt_iter_;
    QLabel* label_wt_analysis_;
    QWidget* wt_curve_widget_;
    QLabel* label_wt_suggest_;

    // 状态
    AutoTunePhase wt_phase_ = AT_IDLE;
    WalkTuneChannel wt_channel_ = WT_CH_DISTANCE;
    int wt_iteration_ = 0;
    bool wt_auto_mode_ = false;
    double wt_baseline_ = 0;
    double wt_target_val_ = 0;
    double wt_step_start_time_ = 0;
    double wt_settle_deadline_ = 0;
    QTimer* wt_timer_ = nullptr;
    std::vector<AutoTuneSample> wt_samples_fwd_;  ///< 正向数据
    std::vector<AutoTuneSample> wt_samples_rev_;  ///< 反向数据
    StepAnalysis wt_result_fwd_;
    StepAnalysis wt_result_rev_;

    void setupTab3_WalkAutoTune(QVBoxLayout* layout);
    void wtStartStep(bool forward);
    void wtSampleTick();
    void wtShowResults();
    void wtApplyNewParams();
    void wtStop();
};

#endif

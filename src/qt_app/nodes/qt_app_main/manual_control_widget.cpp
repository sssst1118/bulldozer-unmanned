/**
 * @file manual_control_widget.cpp
 * @brief 推土机无人驾驶控制台实现
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "manual_control_widget.h"
#include "grid_map_widget.h"
#include "rviz_perception_widget.h"
#include "smooth_ui.h"
#include "squircle.h"
#include "shadow_utils.h"
#include "glass_panel.h"
#include "grouped_section.h"
#include "gesture_handler.h"
#include "qt_app/log_helper.h"
#include "simulink_functions.h"  // [Issue#7] for simulink::INIT_ORIGIN
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QFont>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDir>
#include <QScrollArea>
#include <QScrollBar>
#include <QApplication>
#include <QFrame>
#include <boost/function.hpp>
#include <cmath>
#include <QEvent>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextCursor>
#include <QPixmap>
#include <QIcon>
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QMovie>
#include <QVariantAnimation>
#include <QShortcut>
#include <QGraphicsOpacityEffect>
#include <QGraphicsDropShadowEffect>
#include <QParallelAnimationGroup>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QGuiApplication>
#include <QScreen>
#include <cmath>

// 样式常量 — 特斯拉白色风格 (规则一/四)
// 所有按钮保留语义命名, 颜色值切到特斯拉配色; 字重统一 500 (Medium),
// 圆角统一 8px, 不加阴影/不浮起, hover 仅背景深一级.
// 结构: 先定义 base, 再把 iOS 风的 8 色合并为 5 档语义按钮,
// 原 BLUE/GRAY 直接映射, 原 ORANGE 映射 warn 黄, PINK/PURPLE/CYAN 在特斯拉白里
// 不应该出现, 统一降级为 secondary (白底灰边), 靠图标/文字区分.
static const QString STY_BTN_BASE =
    "QPushButton{border-radius:8px;padding:10px 24px;"
    "font-weight:500;font-size:13px;border:none;}";

static const QString STY_PRIMARY = STY_BTN_BASE +
    "QPushButton{background:#007AFF;color:#FFFFFF;}"
    "QPushButton:hover{background:#0062D4;}"
    "QPushButton:pressed{background:#0055B3;}"
    "QPushButton:disabled{background:#80B9FF;color:#E6F0FF;}";

static const QString STY_SECONDARY = STY_BTN_BASE +
    "QPushButton{background:#FFFFFF;color:#2C2C2E;border:1px solid #E5E7EB;}"
    "QPushButton:hover{background:#F9FAFB;border-color:#D1D1D6;}"
    "QPushButton:pressed{background:#F3F4F6;}"
    "QPushButton:disabled{background:#F9FAFB;color:#D1D1D6;border-color:#F3F4F6;}";

static const QString STY_SUCCESS = STY_BTN_BASE +
    "QPushButton{background:#34C759;color:#FFFFFF;}"
    "QPushButton:hover{background:#2FA350;}"
    "QPushButton:pressed{background:#248A3D;}"
    "QPushButton:disabled{background:#A1E0AF;color:#E8F9EC;}";

static const QString STY_DANGER = STY_BTN_BASE +
    "QPushButton{background:#FF3B30;color:#FFFFFF;}"
    "QPushButton:hover{background:#E0352A;}"
    "QPushButton:pressed{background:#C02825;}"
    "QPushButton:disabled{background:#FFB3AF;color:#FFEEED;}";

static const QString STY_WARN = STY_BTN_BASE +
    "QPushButton{background:#FF9500;color:#FFFFFF;}"
    "QPushButton:hover{background:#E08600;}"
    "QPushButton:pressed{background:#C07300;}"
    "QPushButton:disabled{background:#FFD680;color:#FFF8E6;}";

// 兼容层: 保持旧名称, 映射到新的语义按钮, 避免改动所有调用点
static const QString& STY_GREEN  = STY_SUCCESS;
static const QString& STY_RED    = STY_DANGER;
static const QString& STY_BLUE   = STY_PRIMARY;
static const QString& STY_GRAY   = STY_SECONDARY;
static const QString& STY_ORANGE = STY_WARN;
// 以下三个在特斯拉白风里不应存在, 统一降级为 secondary
static const QString& STY_PINK   = STY_SECONDARY;
static const QString& STY_PURPLE = STY_SECONDARY;
static const QString& STY_CYAN   = STY_SECONDARY;

// 数值等宽字体 (规则二) + 侧栏标签字体
static const QFont MONO_FONT = SmoothUI::valueFont(10);
static const QFont SIDE_FONT = SmoothUI::labelFont(11);

//==============================================================================
// loadPngPixmap — 强制 PNG 解码器加载 qrc 中的 PNG 资源
//   [Fix] 某些 Linux 发行版上 QPixmap 直接构造从 qrc 加载 PNG 会静默返回 null,
//         原因疑为 Qt 对 imageformats 插件的探测路径异常 (WSL/Windows Qt 不受影响).
//         显式走 QImage::fromData(..., "PNG") 绕过自动格式识别, 强制内置 PNG 解码.
//==============================================================================
static QPixmap loadPngPixmap(const QString& resourcePath) {
    QFile f(resourcePath);
    if (!f.open(QIODevice::ReadOnly)) return QPixmap();
    QImage img = QImage::fromData(f.readAll(), "PNG");
    return QPixmap::fromImage(img);
}

//==============================================================================
// SmartSpinBox — 带范围色带 + 修饰键加速的增强型 double spin box
//==============================================================================
SmartSpinBox::SmartSpinBox(QWidget* parent) : QDoubleSpinBox(parent) {
    // 特斯拉白配色 (规则一/四/十一): 白底 + 极浅灰边框 + focus 蓝边.
    // 底部多 8px 给色带, padding-bottom 比 top 多出相应距离.
    // [hasWarning="true"] 规则 (规则十一): 值超出 setSafeRange() 时边框变红 + 浅红底.
    setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    // Qt 5 的 QString::arg() 多参数形式上限 9 个, 所以拆成两段 setStyleSheet 拼接.
    const QString qssBase = QStringLiteral(
        "SmartSpinBox{background:%1;color:%2;"
        "border:1px solid %3;border-radius:%4px;"
        "padding:8px 12px 16px 12px;"
        "font-family:\"JetBrains Mono\",\"Consolas\",monospace;}"
        "SmartSpinBox:hover{border-color:%5;}"
        "SmartSpinBox:focus{border-color:%6;}"
        "SmartSpinBox:disabled{background:%7;color:%8;border-color:%9;}"
    ).arg(Theme::hex(Theme::bgInput),            // %1
          Theme::hex(Theme::textPrimary),         // %2
          Theme::hex(Theme::borderDefault),       // %3
          QString::number(Spacing::inputRadius),  // %4
          Theme::hex(Theme::borderHover),         // %5
          Theme::hex(Theme::borderFocus),         // %6
          Theme::hex(Theme::bgSurface),           // %7
          Theme::hex(Theme::textDisabled),        // %8
          Theme::hex(Theme::bgHover));            // %9
    const QString qssWarn = QStringLiteral(
        "SmartSpinBox[hasWarning=\"true\"]{border-color:%1;background:%2;}"
        "SmartSpinBox[hasWarning=\"true\"]:focus{border-color:%1;}"
    ).arg(Theme::hex(Theme::colorError),
          Theme::hex(Theme::colorErrorBg));
    setStyleSheet(qssBase + qssWarn);
    setProperty("hasWarning", false);
}

void SmartSpinBox::setSafeRange(double lo, double hi) {
    has_safe_ = true;
    safe_lo_ = lo; safe_hi_ = hi;
    // 连接一次 valueChanged 驱动警告检查; 用 UniqueConnection 防重
    connect(this, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SmartSpinBox::checkSafeRange, Qt::UniqueConnection);
    checkSafeRange(value());
}

void SmartSpinBox::clearSafeRange() {
    has_safe_ = false;
    if (currently_unsafe_) {
        currently_unsafe_ = false;
        setProperty("hasWarning", false);
        style()->unpolish(this); style()->polish(this); update();
    }
}

void SmartSpinBox::checkSafeRange(double v) {
    if (!has_safe_) return;
    bool unsafe = (v < safe_lo_ || v > safe_hi_);
    if (unsafe != currently_unsafe_) {
        currently_unsafe_ = unsafe;
        setProperty("hasWarning", unsafe);
        // 切换 dynamic property 后必须 unpolish+polish 才触发 QSS 重算
        style()->unpolish(this); style()->polish(this); update();
        // 进入警告时左右抖一下 (iOS 融合风规则十一 + 补充四)
        if (unsafe) {
            SmoothUI::shakeWidget(this, 6, 400);
        }
    }
}

void SmartSpinBox::focusInEvent(QFocusEvent* e) {
    QDoubleSpinBox::focusInEvent(e);
    SmoothUI::animateProperty(this, "focusRingOpacity", 1.0, 200,
                              QEasingCurve::OutCubic);
}

void SmartSpinBox::focusOutEvent(QFocusEvent* e) {
    QDoubleSpinBox::focusOutEvent(e);
    SmoothUI::animateProperty(this, "focusRingOpacity", 0.0, 250,
                              QEasingCurve::InQuad);
}

QSize SmartSpinBox::sizeHint() const {
    QSize s = QDoubleSpinBox::sizeHint();
    return QSize(s.width(), s.height() + 8);        // 底部多 8px 给色带
}
QSize SmartSpinBox::minimumSizeHint() const {
    QSize s = QDoubleSpinBox::minimumSizeHint();
    return QSize(s.width(), s.height() + 8);
}

void SmartSpinBox::paintEvent(QPaintEvent* e) {
    QDoubleSpinBox::paintEvent(e);                  // 原生 spin (文本框 + 上下箭头)

    // ---- 底部可视化色带 (配色与 Theme 对齐, 特斯拉白风) ----
    // 提前构造 QColor 避免每次 paintEvent 解析十六进制字符串;
    // 关闭抗锯齿 — 色带仅 3px 高, 抗锯齿反而让像素发灰, 且拖动时更耗.
    static const QColor COL_TRACK     = Theme::sliderTrack;      // 轨道浅灰
    static const QColor COL_REC       = QColor(Theme::colorOk.red(),
                                               Theme::colorOk.green(),
                                               Theme::colorOk.blue(), 140);
    static const QColor COL_CRIT      = QColor(Theme::colorError.red(),
                                               Theme::colorError.green(),
                                               Theme::colorError.blue(), 130);
    static const QColor COL_DOT       = Theme::accentBlue;       // 当前值蓝点
    static const QColor COL_DOT_INNER = Theme::bgBase;           // 内芯白

    const double mn = minimum(), mx = maximum();
    if (mx <= mn) return;

    QPainter p(this);
    const int barH = 3;
    const int barY = height() - barH - 3;
    const int barL = 6, barR = width() - 6;
    const int rangeW = barR - barL;

    auto toX = [&](double v) {
        double norm = qBound(0.0, (v - mn) / (mx - mn), 1.0);
        return barL + static_cast<int>(norm * rangeW);
    };

    p.setPen(Qt::NoPen);

    // 1. 灰色轨道 (整数矩形, 省 QRectF 浮点)
    p.setBrush(COL_TRACK);
    p.drawRect(barL, barY, rangeW, barH);

    // 2. 推荐范围
    if (has_rec_) {
        int x1 = toX(rec_lo_), x2 = toX(rec_hi_);
        p.setBrush(COL_REC);
        p.drawRect(x1, barY, x2 - x1, barH);
    }

    // 3. 临界范围
    if (has_crit_) {
        int x1 = toX(crit_lo_), x2 = toX(crit_hi_);
        p.setBrush(COL_CRIT);
        p.drawRect(x1, barY, x2 - x1, barH);
    }

    // 4. 当前值指示点 — 小圆才用抗锯齿, 开关局部化
    int px = toX(value());
    int cy = barY + barH / 2;
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setBrush(COL_DOT);
    p.drawEllipse(QPoint(px, cy), 5, 5);
    p.setBrush(COL_DOT_INNER);
    p.drawEllipse(QPoint(px, cy), 2, 2);

    // 5. Focus 内圈光晕 (iOS 融合风规则七): focus 时从内部 2px 画一圈半透明 iOS 蓝,
    //    配合 QSS 的 border-color 变蓝, 形成"实线蓝边 + 浅蓝内圈"的双层聚焦提示.
    //    圆角用 Squircle 超椭圆, 视觉上比标准圆角更柔和连续.
    if (focus_ring_opacity_ > 0.01) {
        int alpha = static_cast<int>(55 * focus_ring_opacity_);
        QColor ring(Theme::borderFocus.red(),
                    Theme::borderFocus.green(),
                    Theme::borderFocus.blue(), alpha);
        p.setPen(QPen(ring, 2));
        p.setBrush(Qt::NoBrush);
        p.drawPath(Squircle::path(QRectF(rect().adjusted(2, 2, -3, -3)),
                                  Spacing::inputRadius - 1));
    }
}

void SmartSpinBox::wheelEvent(QWheelEvent* e) {
    if (!hasFocus()) { e->ignore(); return; }       // 保持原"无焦点不响应滚轮"的安全策略
    double step = singleStep();
    if (e->modifiers() & Qt::ShiftModifier)      step *= 10;   // 粗调
    else if (e->modifiers() & Qt::AltModifier)   step /= 10;   // 细调
    int dir = (e->angleDelta().y() > 0) ? 1 : -1;
    setValue(value() + dir * step);
    e->accept();
}

//==============================================================================
// SmoothSlider — 特斯拉风自绘水平滑块 (规则 6.1)
//==============================================================================
SmoothSlider::SmoothSlider(Qt::Orientation orientation, QWidget* parent)
    : QSlider(orientation, parent) {
    setMinimumHeight(28);
    setCursor(Qt::PointingHandCursor);
    // 禁用 QStyle, 完全走自绘 paintEvent
    setStyleSheet("SmoothSlider{background:transparent;border:none;}");
}

void SmoothSlider::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    int w = width(), h = height();
    double range = (maximum() - minimum());
    double ratio = range > 0 ? (double(value() - minimum()) / range) : 0.0;
    double trackY = h / 2.0;
    double trackH = 4.0;                         // iOS 融合风: 略粗一点更饱满
    int L = 8, R = w - 8;                        // 左右留 8px 给滑块
    double fillX = L + ratio * (R - L);

    // 轨道背景 (Squircle 胶囊)
    Squircle::draw(p, QRectF(L, trackY - trackH / 2.0, R - L, trackH),
                   trackH / 2.0, QBrush(Theme::sliderTrack));

    // 已填充 (iOS 标准蓝)
    if (fillX > L) {
        Squircle::draw(p, QRectF(L, trackY - trackH / 2.0, fillX - L, trackH),
                       trackH / 2.0, QBrush(Theme::sliderFill));
    }

    // 滑块阴影 (两层叠加, iOS 饱满质感)
    double thumbR = 9.0 * thumb_scale_;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 10));
    p.drawEllipse(QPointF(fillX, trackY + 1), thumbR + 1, thumbR + 1);
    p.setBrush(QColor(0, 0, 0, 18));
    p.drawEllipse(QPointF(fillX, trackY + 0.5), thumbR + 0.5, thumbR + 0.5);
    // 白底 + iOS 蓝描边 (规则 6.1)
    p.setBrush(Theme::bgBase);
    p.setPen(QPen(Theme::sliderFill, 1.5));
    p.drawEllipse(QPointF(fillX, trackY), thumbR, thumbR);
}

void SmoothSlider::enterEvent(QEvent* e) {
    QSlider::enterEvent(e);
    SmoothUI::animateProperty(this, "thumbScale", 1.15, 180, QEasingCurve::OutCubic);
}

void SmoothSlider::leaveEvent(QEvent* e) {
    QSlider::leaveEvent(e);
    SmoothUI::animateProperty(this, "thumbScale", 1.0, 200, QEasingCurve::OutCubic);
}

//==============================================================================
// SmoothToggle — iOS 标准拨动开关 (融合风规则 9 / 第九部分)
//   - 尺寸 51×31 (iOS 标准, 更饱满)
//   - 关闭态轨道 #E5E5EA, 开启态轨道绿色 #34C759 (iOS 开关是绿的, 不是蓝)
//   - 切换用 springAnimate (OutBack overshoot=1.5), 有轻微回弹
//   - 颜色用 animateColor 平滑过渡 (颜色永远不用弹簧)
//==============================================================================
SmoothToggle::SmoothToggle(QWidget* parent)
    : QAbstractButton(parent), track_color_(Theme::toggleOff) {
    setCheckable(true);
    setFixedSize(Spacing::toggleWidth, Spacing::toggleHeight);  // 51 × 31
    setCursor(Qt::PointingHandCursor);
}

void SmoothToggle::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    int h = height(), w = width();

    // 轨道: 用 Squircle 胶囊 (圆角 = h/2)
    Squircle::draw(p, QRectF(0, 0, w, h), h / 2.0, QBrush(track_color_));

    // 滑块: 双层阴影 + 白圆 (iOS 饱满质感); hover 时 thumbScale 微放大
    double thumbD = (h - 4) * thumb_scale_;
    double cx = thumb_x_ + (h - 4) / 2.0;   // 中心保持在原位置
    double cy = h / 2.0;
    p.setPen(Qt::NoPen);
    // 第一层: 贴合阴影
    p.setBrush(QColor(0, 0, 0, 10));
    p.drawEllipse(QPointF(cx, cy + 0.5), thumbD / 2.0 + 0.5, thumbD / 2.0 + 0.5);
    // 第二层: 柔和阴影
    p.setBrush(QColor(0, 0, 0, 24));
    p.drawEllipse(QPointF(cx, cy + 1.5), thumbD / 2.0 + 0.3, thumbD / 2.0 + 0.3);
    // 白色圆
    p.setBrush(Theme::bgBase);
    p.drawEllipse(QPointF(cx, cy), thumbD / 2.0, thumbD / 2.0);
}

void SmoothToggle::checkStateSet() {
    // 每次 setChecked(...) 或点击切换都走这里, 不需要覆写 nextCheckState:
    // - 鼠标/空格触发 click() -> 默认 nextCheckState() -> setChecked() -> 本函数
    // - 外部调 setChecked() -> 本函数
    // toggled(bool) 信号由 QAbstractButton::setChecked() 自己 emit, 无需重复.
    QAbstractButton::checkStateSet();
    animateToState();
}

void SmoothToggle::resizeEvent(QResizeEvent* e) {
    QAbstractButton::resizeEvent(e);
    // 无动画时保持滑块在正确端, 避免初始化时 thumb_x_ 还是 2.0 但 checked=true
    thumb_x_ = isChecked() ? (width() - height() + 2.0) : 2.0;
}

void SmoothToggle::animateToState() {
    double targetX = isChecked() ? (width() - height() + 2.0) : 2.0;
    QColor targetTrack = isChecked() ? Theme::toggleOn : Theme::toggleOff;
    // iOS 开关标志性弹簧: OutBack + overshoot=1.5, 有明显回弹
    SmoothUI::springAnimate(this, "thumbX", targetX, 350, 1.5);
    // 颜色平滑过渡 (不用弹簧)
    SmoothUI::animateColor(this, "trackColor", targetTrack, 200);
}

void SmoothToggle::enterEvent(QEvent* e) {
    QAbstractButton::enterEvent(e);
    SmoothUI::smoothAnimate(this, "thumbScale", 1.08, 160);
}

void SmoothToggle::leaveEvent(QEvent* e) {
    QAbstractButton::leaveEvent(e);
    SmoothUI::smoothAnimate(this, "thumbScale", 1.0, 220);
}

//==============================================================================
// SideDot — 侧边栏状态圆点 (自绘, 颜色动画过渡)
//==============================================================================
SideDot::SideDot(QWidget* parent)
    : QWidget(parent), dot_color_(Theme::textDisabled) {
    setFixedSize(10, 10);
}

void SideDot::animateTo(const QColor& c) {
    SmoothUI::animateColor(this, "dotColor", c, 300);
}

void SideDot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(Qt::NoPen);
    p.setBrush(dot_color_);
    // 内缩 0.5px 防止抗锯齿溢出 widget 边界
    p.drawEllipse(QRectF(0.5, 0.5, width() - 1, height() - 1));
}

//==============================================================================
// SegmentedControl — iOS 分段控制器 (融合风第八部分)
//==============================================================================
SegmentedControl::SegmentedControl(const QStringList& segments, QWidget* parent)
    : QWidget(parent), segments_(segments) {
    setFixedHeight(32);
    setCursor(Qt::PointingHandCursor);
    setAttribute(Qt::WA_Hover, true);
}

void SegmentedControl::resizeEvent(QResizeEvent*) {
    if (segments_.isEmpty()) return;
    double segW = (width() - 4.0) / segments_.size();
    thumb_width_ = segW;
    thumb_x_ = 2.0 + selected_index_ * segW;
    update();
}

void SegmentedControl::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    if (segments_.isEmpty()) return;

    int h = height();

    // 底部轨道 (iOS 分段控制器半透明灰)
    Squircle::draw(p, QRectF(rect()), h / 2.0,
                   QBrush(Theme::glassSegment()));

    // 选中滑块 (白色 + 微阴影)
    QRectF thumbRect(thumb_x_, 2, thumb_width_, h - 4);
    // 滑块阴影
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 18));
    QPainterPath shadowPath = Squircle::path(
        thumbRect.adjusted(0, 1, 0, 1), (h - 4) / 2.0);
    p.drawPath(shadowPath);
    // 滑块本体
    Squircle::draw(p, thumbRect, (h - 4) / 2.0, QBrush(Theme::bgBase));

    // 文字标签 (选中黑色, 其它灰色)
    double segW = (width() - 4.0) / segments_.size();
    p.setFont(SmoothUI::labelFont(12));
    for (int i = 0; i < segments_.size(); i++) {
        QRectF textRect(2 + i * segW, 0, segW, h);
        p.setPen(i == selected_index_ ? Theme::textPrimary : Theme::textSecondary);
        p.drawText(textRect, Qt::AlignCenter, segments_[i]);
    }
}

void SegmentedControl::setSelectedIndex(int index) {
    if (index == selected_index_ || index < 0 || index >= segments_.size())
        return;
    selected_index_ = index;
    double segW = (width() - 4.0) / segments_.size();
    double targetX = 2.0 + index * segW;
    // iOS 标志性弹簧动画, overshoot 适中
    SmoothUI::springAnimate(this, "thumbX", targetX, 350, 1.0);
    update();
    emit selectionChanged(index);
}

void SegmentedControl::mousePressEvent(QMouseEvent* e) {
    if (segments_.isEmpty()) return;
    double segW = (width() - 4.0) / segments_.size();
    int index = static_cast<int>((e->pos().x() - 2) / segW);
    index = qBound(0, index, int(segments_.size() - 1));
    setSelectedIndex(index);
}

//==============================================================================
// IOSButton — iOS 按压缩放按钮 (融合风第十部分) — 支持 4 种 Variant 颜色
//==============================================================================
namespace {
// Variant 配色规格: 3 态背景 + 文字色 + 可选边框 (仅 Secondary 有边框)
struct VariantColors {
    QColor normal, pressed, disabled;   // 背景 3 态
    QColor text;                         // 文字色 (enabled)
    QColor border;                       // 边框 (invalid = 不画)
};
inline VariantColors colorsOf(IOSButton::Variant v) {
    switch (v) {
        case IOSButton::Success:
            return { Theme::colorOk,    QColor(0x24, 0x8A, 0x3D), QColor(0xA1, 0xE0, 0xAF),
                     Qt::white, QColor() };
        case IOSButton::Danger:
            return { Theme::colorError, QColor(0xC0, 0x28, 0x25), QColor(0xFF, 0xB3, 0xAF),
                     Qt::white, QColor() };
        case IOSButton::Warn:
            return { Theme::colorWarn,  QColor(0xC0, 0x73, 0x00), QColor(0xFF, 0xD6, 0x80),
                     Qt::white, QColor() };
        case IOSButton::Secondary:
            // 次要按钮: 白底 + 浅灰边 + 深色字, hover 底色变浅灰
            return { Theme::bgBase, Theme::bgHover, Theme::bgSurface,
                     Theme::textPrimary, Theme::borderDefault };
        case IOSButton::Primary:
        default:
            return { Theme::accentBlue, QColor(0x00, 0x55, 0xB3), QColor(0x80, 0xB9, 0xFF),
                     Qt::white, QColor() };
    }
}
} // anonymous namespace

IOSButton::IOSButton(const QString& text, Variant variant, QWidget* parent)
    : QPushButton(text, parent), variant_(variant) {
    setCursor(Qt::PointingHandCursor);
    setMinimumHeight(44);
    // 屏蔽默认 style, 我们完全自绘
    setStyleSheet("QPushButton{background:transparent;border:none;}");
}

void IOSButton::mousePressEvent(QMouseEvent* e) {
    QPushButton::mousePressEvent(e);
    // 按下快速缩小到 0.97, 用平滑动画 (非弹簧)
    SmoothUI::smoothAnimate(this, "pressScale", 0.97, 100);
}

void IOSButton::mouseReleaseEvent(QMouseEvent* e) {
    QPushButton::mouseReleaseEvent(e);
    // 松开弹回 1.0, 用弹簧带明显回弹
    SmoothUI::springAnimate(this, "pressScale", 1.0, 300, 1.5);
}

void IOSButton::enterEvent(QEvent* e) {
    QPushButton::enterEvent(e);
    // hover 阴影渐增强 (alpha 18→42, offset 2→4, blur 6→11)
    SmoothUI::smoothAnimate(this, "hoverIntensity", 1.0, 180);
}

void IOSButton::leaveEvent(QEvent* e) {
    QPushButton::leaveEvent(e);
    SmoothUI::smoothAnimate(this, "hoverIntensity", 0.0, 220);
}

void IOSButton::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // 应用缩放 (以按钮中心为原点)
    p.translate(width() / 2.0, height() / 2.0);
    p.scale(press_scale_, press_scale_);
    p.translate(-width() / 2.0, -height() / 2.0);

    QRectF r = rect();

    // 阴影 (近松开时才画, 避免按下时阴影跟着缩).
    // hover 时阴影参数增强: offset y 2→4, blur 6→11, alpha 18→42 — iOS 卡片悬停抬起感.
    if (press_scale_ >= 0.995) {
        double h = hover_intensity_;
        int yOff  = static_cast<int>(2 + 2 * h);
        int blur  = static_cast<int>(6 + 5 * h);
        int alpha = static_cast<int>(18 + 24 * h);
        ShadowUtils::drawShadow(p, rect().adjusted(0, yOff, 0, yOff),
            Spacing::buttonRadius, QColor(0, 0, 0, alpha), 0, 0, blur);
    }

    // 按钮本体 (Squircle 圆角 + Variant 对应色)
    VariantColors vc = colorsOf(variant_);
    QColor bg = isEnabled() ? (isDown() ? vc.pressed : vc.normal) : vc.disabled;
    Squircle::draw(p, r, Spacing::buttonRadius, QBrush(bg));

    // 边框 (仅 Secondary 变体有浅灰边; border 无效时跳过)
    if (vc.border.isValid()) {
        p.setPen(QPen(vc.border, 1));
        p.setBrush(Qt::NoBrush);
        p.drawPath(Squircle::path(r, Spacing::buttonRadius));
    }

    // 文字: Variant 自己的 text 色, disabled 时淡化
    p.setPen(isEnabled() ? vc.text : Theme::textDisabled);
    p.setFont(SmoothUI::buttonFont());
    p.drawText(r, Qt::AlignCenter, text());
}

//==============================================================================
// SuccessHUD — iOS 风成功 HUD (融合风补充五)
//==============================================================================
SuccessHUD::SuccessHUD(QWidget* parent, const QString& text)
    : QWidget(parent), text_(text)
{
    setFixedSize(120, 120);
    // 居中于父控件
    if (parent) {
        move((parent->width() - 120) / 2, (parent->height() - 120) / 2);
    }
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_DeleteOnClose);

    // 勾号进度计时器, 60fps 大约 6 帧画完
    check_timer_ = new QTimer(this);
    connect(check_timer_, &QTimer::timeout, this, [this]() {
        check_progress_ += 0.06;
        if (check_progress_ >= 1.0) {
            check_progress_ = 1.0;
            check_timer_->stop();
        }
        update();
    });

    QWidget::show();
    raise();
}

void SuccessHUD::show(QWidget* parent, const QString& text) {
    if (!parent) return;
    auto* hud = new SuccessHUD(parent, text);
    hud->animate();
}

void SuccessHUD::animate() {
    // 弹入: 0.8 → 1.0 弹簧缩放 + 0.0 → 1.0 平滑渐入
    SmoothUI::springAnimate(this, "hudScale", 1.0, 400, 1.2);
    SmoothUI::smoothAnimate(this, "hudOpacity", 1.0, 200);

    // 延迟 200ms 画勾, 给 HUD 弹出留出视觉停留时间
    QTimer::singleShot(200, this, [this]() {
        if (check_timer_) check_timer_->start(16);
    });

    // 1800ms 后渐出并销毁
    QTimer::singleShot(1800, this, [this]() {
        SmoothUI::smoothAnimate(this, "hudOpacity", 0.0, 300);
        SmoothUI::smoothAnimate(this, "hudScale", 0.9, 300);
        QTimer::singleShot(350, this, &QWidget::deleteLater);
    });
}

void SuccessHUD::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setOpacity(hud_opacity_);

    // 以中心为原点缩放
    p.translate(60, 60);
    p.scale(hud_scale_, hud_scale_);
    p.translate(-60, -60);

    // 背景: 深色半透明 iOS HUD 圆角
    Squircle::draw(p, QRectF(0, 0, 120, 120), 24,
                   QBrush(QColor(30, 30, 30, 200)));

    // 勾号 (分两段绘制: 起点→拐点, 拐点→终点, 随 check_progress_ 推进)
    if (check_progress_ > 0.0) {
        QPen checkPen(Qt::white, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        p.setPen(checkPen);
        QPointF p1(35, 62);   // 起点
        QPointF p2(52, 78);   // 拐点
        QPointF p3(85, 45);   // 终点

        QPainterPath checkPath;
        checkPath.moveTo(p1);
        if (check_progress_ <= 0.4) {
            double t = check_progress_ / 0.4;
            QPointF mid = p1 + (p2 - p1) * t;
            checkPath.lineTo(mid);
        } else {
            checkPath.lineTo(p2);
            double t = (check_progress_ - 0.4) / 0.6;
            QPointF end = p2 + (p3 - p2) * t;
            checkPath.lineTo(end);
        }
        p.drawPath(checkPath);
    }

    // 底部文字 (淡入跟随勾号进度)
    int textAlpha = static_cast<int>(220 * check_progress_);
    p.setPen(QColor(255, 255, 255, textAlpha));
    p.setFont(SmoothUI::labelFont(12));
    p.drawText(QRect(0, 88, 120, 24), Qt::AlignCenter, text_);
}

//==============================================================================
// 阶段三 · 布局系统 (融合风补充篇·补充一/二/十)
//==============================================================================

//------------------------------------------------------------------------------
// ParameterRow — iOS 单行参数容器
//------------------------------------------------------------------------------
ParameterRow::ParameterRow(const QString& label, QWidget* control, QWidget* parent)
    : QWidget(parent),
      control_(control),
      bg_color_(Qt::transparent)
{
    setFixedHeight(Spacing::cellHeight + 8);     // 52 px
    setAttribute(Qt::WA_Hover, true);

    row_layout_ = new QHBoxLayout(this);
    row_layout_->setContentsMargins(20, 0, 20, 0);
    row_layout_->setSpacing(12);

    // 左侧标签列 (主标签 + 可选灰色 hint)
    label_col_ = new QVBoxLayout();
    label_col_->setSpacing(0);
    label_col_->setContentsMargins(0, 0, 0, 0);

    lbl_ = new QLabel(label, this);
    lbl_->setFont(SmoothUI::labelFont(12));
    lbl_->setStyleSheet(QStringLiteral(
        "QLabel{color:%1;background:transparent;}").arg(Theme::hex(Theme::textPrimary)));

    label_col_->addWidget(lbl_);
    row_layout_->addLayout(label_col_);
    row_layout_->addStretch();

    if (control_) {
        row_layout_->addWidget(control_);
    }
}

ParameterRow::ParameterRow(QWidget* labelWidget, QWidget* control, QWidget* parent)
    : QWidget(parent),
      control_(control),
      bg_color_(Qt::transparent)
{
    setFixedHeight(Spacing::cellHeight + 8);     // 52 px
    setAttribute(Qt::WA_Hover, true);
    row_layout_ = new QHBoxLayout(this);
    row_layout_->setContentsMargins(20, 0, 20, 0);
    row_layout_->setSpacing(12);
    label_col_ = new QVBoxLayout();
    label_col_->setSpacing(0);
    label_col_->setContentsMargins(0, 0, 0, 0);
    if (labelWidget) {
        if (labelWidget->parent() != this) labelWidget->setParent(this);
        label_col_->addWidget(labelWidget);
    }
    row_layout_->addLayout(label_col_);
    row_layout_->addStretch();
    if (control_) row_layout_->addWidget(control_);
}

void ParameterRow::setUnit(const QString& unit) {
    if (!unit_lbl_) {
        unit_lbl_ = new QLabel(this);
        unit_lbl_->setFont(SmoothUI::unitFont(10));
        unit_lbl_->setStyleSheet(QStringLiteral(
            "QLabel{color:%1;background:transparent;}")
            .arg(Theme::hex(Theme::textSecondary)));
        // 单位显示在 control_ 右侧
        row_layout_->addWidget(unit_lbl_);
    }
    unit_lbl_->setText(unit);
}

void ParameterRow::setHint(const QString& hint) {
    if (!hint_lbl_) {
        hint_lbl_ = new QLabel(this);
        hint_lbl_->setFont(SmoothUI::unitFont(9));
        hint_lbl_->setStyleSheet(QStringLiteral(
            "QLabel{color:%1;background:transparent;}")
            .arg(Theme::hex(Theme::textSecondary)));
        label_col_->addWidget(hint_lbl_);
    }
    hint_lbl_->setText(hint);
}

void ParameterRow::setControl(QWidget* control) {
    if (control_) {
        row_layout_->removeWidget(control_);
        control_->deleteLater();
    }
    control_ = control;
    if (control_) {
        // 把新控件插入到 stretch 之后, unit_lbl_ 之前
        int insertAt = row_layout_->count();
        if (unit_lbl_) {
            for (int i = 0; i < row_layout_->count(); ++i) {
                if (row_layout_->itemAt(i)->widget() == unit_lbl_) {
                    insertAt = i;
                    break;
                }
            }
        }
        row_layout_->insertWidget(insertAt, control_);
    }
}

void ParameterRow::paintEvent(QPaintEvent*) {
    if (bg_color_.alpha() == 0) return;
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), bg_color_);
}

void ParameterRow::enterEvent(QEvent*) {
    SmoothUI::animateColor(this, "bgColor", Theme::bgHover, 180);
}

void ParameterRow::leaveEvent(QEvent*) {
    SmoothUI::animateColor(this, "bgColor", QColor(0, 0, 0, 0), 180);
}

//------------------------------------------------------------------------------
// ScrollIndicator — 自动隐藏的细滚动条
//------------------------------------------------------------------------------
ScrollIndicator::ScrollIndicator(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    fade_timer_ = new QTimer(this);
    fade_timer_->setSingleShot(true);
    connect(fade_timer_, &QTimer::timeout, this, [this]() {
        SmoothUI::animateProperty(this, "opacity", 0.0, 350,
                                  QEasingCurve::OutCubic);
    });
}

void ScrollIndicator::notifyScroll(int contentY, int contentH, int viewportH) {
    if (contentH <= viewportH) {
        opacity_ = 0.0;
        update();
        return;
    }
    int trackH = viewportH - 16;            // 上下各 8px 留白
    double ratio = double(viewportH) / contentH;
    int barH = qMax(28, int(trackH * ratio));
    int maxOff = trackH - barH;
    double pos = double(contentY) / qMax(1, contentH - viewportH);
    pos = qBound(0.0, pos, 1.0);
    int barY = 8 + int(maxOff * pos);
    bar_rect_ = QRect(width() - 6, barY, 3, barH);
    opacity_ = 1.0;
    update();
    fade_timer_->start(1500);
}

void ScrollIndicator::paintEvent(QPaintEvent*) {
    if (opacity_ <= 0.01 || bar_rect_.isEmpty()) return;
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    QColor c = Theme::textMuted;
    c.setAlphaF(qBound(0.0, 0.55 * opacity_, 1.0));
    p.setBrush(c);
    p.setPen(Qt::NoPen);
    p.drawRoundedRect(bar_rect_, 1.5, 1.5);
}

//------------------------------------------------------------------------------
// RubberScrollArea — 弹性滚动 (顶/底阻尼)
//------------------------------------------------------------------------------
RubberScrollArea::RubberScrollArea(QWidget* parent)
    : QScrollArea(parent)
{
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setFrameShape(QFrame::NoFrame);
    setWidgetResizable(true);
    indicator_ = new ScrollIndicator(this);
    indicator_->raise();
}

void RubberScrollArea::setRubberOffset(int v) {
    rubber_offset_ = v;
    if (widget()) {
        widget()->move(content_origin_.x(), content_origin_.y() + v);
    }
}

void RubberScrollArea::wheelEvent(QWheelEvent* e) {
    QScrollBar* vbar = verticalScrollBar();
    int before = vbar->value();
    QScrollArea::wheelEvent(e);
    int after = vbar->value();

    bool atTop    = (after == vbar->minimum());
    bool atBottom = (after == vbar->maximum());

    // 如果滚动到了边界且滚轮还有剩余 → 进入橡皮筋模式
    if (atTop || atBottom) {
        int dy = e->angleDelta().y();
        if ((atTop && dy > 0) || (atBottom && dy < 0)) {
            constexpr int RESISTANCE = 3;
            constexpr int LIMIT = 80;
            int add = dy / (8 * RESISTANCE);    // 滚轮单位 ~120 → ~5 px
            int newOff = qBound(-LIMIT, rubber_offset_ + add, LIMIT);
            if (widget()) {
                if (content_origin_.isNull() || before == after) {
                    content_origin_ = widget()->pos();
                }
                content_origin_ = QPoint(widget()->x(),
                                         widget()->y() - rubber_offset_);
            }
            setRubberOffset(newOff);
            // 滚轮停止 250ms 后自动弹回
            QTimer::singleShot(180, this, [this]() {
                if (qAbs(rubber_offset_) > 0) springBack();
            });
            e->accept();
        }
    }
    updateIndicator();
}

void RubberScrollArea::resizeEvent(QResizeEvent* e) {
    QScrollArea::resizeEvent(e);
    indicator_->setGeometry(0, 0, width(), height());
    updateIndicator();
}

void RubberScrollArea::scrollContentsBy(int dx, int dy) {
    QScrollArea::scrollContentsBy(dx, dy);
    if (widget()) content_origin_ = widget()->pos();
    updateIndicator();
}

void RubberScrollArea::springBack() {
    if (spring_running_) return;
    spring_running_ = true;
    SmoothUI::springAnimate(this, "rubberOffset", 0, 350, 1.2);
    QTimer::singleShot(380, this, [this]() { spring_running_ = false; });
}

void RubberScrollArea::updateIndicator() {
    if (!widget() || !indicator_) return;
    indicator_->notifyScroll(verticalScrollBar()->value(),
                             widget()->height(),
                             viewport()->height());
}

//------------------------------------------------------------------------------
// StickyFooter — 底部固定操作栏
//------------------------------------------------------------------------------
StickyFooter::StickyFooter(const QString& applyText, QWidget* parent)
    : QWidget(parent)
{
    setFixedHeight(90);
    auto* lay = new QHBoxLayout(this);
    lay->setContentsMargins(20, 24, 20, 18);
    apply_btn_ = new IOSButton(applyText, IOSButton::Primary, this);
    apply_btn_->setMinimumHeight(44);
    apply_btn_->setMinimumWidth(160);
    lay->addStretch();
    lay->addWidget(apply_btn_);
    connect(apply_btn_, &QPushButton::clicked, this, &StickyFooter::applyClicked);
}

void StickyFooter::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, false);

    // 顶部 12 px 渐隐遮罩 (透明 → bgWindow)
    QLinearGradient grad(0, 0, 0, 12);
    grad.setColorAt(0.0, QColor(Theme::bgWindow.red(),
                                 Theme::bgWindow.green(),
                                 Theme::bgWindow.blue(), 0));
    grad.setColorAt(1.0, Theme::bgWindow);
    p.fillRect(0, 0, width(), 12, grad);

    // 下方实色 bgWindow
    p.fillRect(0, 12, width(), height() - 12, Theme::bgWindow);
}

//==============================================================================
// 阶段四 · 动效打磨 (融合风补充篇·补充三/六/七/八)
//==============================================================================

//------------------------------------------------------------------------------
// StaggerAnimator — 列表项依次入场动画 (补充三)
//------------------------------------------------------------------------------
void StaggerAnimator::animateIn(const QList<QWidget*>& widgets,
                                int staggerDelay, int duration) {
    for (int i = 0; i < widgets.size(); ++i) {
        QWidget* w = widgets[i];
        if (!w) continue;

        // 透明度: 复用已有 QGraphicsOpacityEffect, 没有则创建
        auto* eff = qobject_cast<QGraphicsOpacityEffect*>(w->graphicsEffect());
        if (!eff) {
            eff = new QGraphicsOpacityEffect(w);
            w->setGraphicsEffect(eff);
        }
        eff->setOpacity(0.0);

        // 位移: 记录原位 + 临时下移 20px (起始态)
        const QPoint origin = w->pos();
        w->move(origin.x(), origin.y() + 20);

        QPointer<QWidget> guard(w);
        QPointer<QGraphicsOpacityEffect> effGuard(eff);
        QTimer::singleShot(i * staggerDelay, w,
            [guard, effGuard, origin, duration]() {
                if (!guard || !effGuard) return;
                // 透明度渐入
                auto* fade = new QPropertyAnimation(effGuard.data(), "opacity");
                fade->setDuration(duration);
                fade->setStartValue(0.0);
                fade->setEndValue(1.0);
                fade->setEasingCurve(QEasingCurve::OutCubic);
                // 位移弹回原位
                auto* slide = new QPropertyAnimation(guard.data(), "pos");
                slide->setDuration(duration);
                slide->setStartValue(guard->pos());
                slide->setEndValue(origin);
                slide->setEasingCurve(QEasingCurve::OutCubic);
                auto* group = new QParallelAnimationGroup(guard.data());
                group->addAnimation(fade);
                group->addAnimation(slide);
                group->start(QAbstractAnimation::DeleteWhenStopped);
            });
    }
}

void StaggerAnimator::animateOut(const QList<QWidget*>& widgets,
                                 int staggerDelay, int duration) {
    for (int i = 0; i < widgets.size(); ++i) {
        QWidget* w = widgets[i];
        if (!w) continue;
        auto* eff = qobject_cast<QGraphicsOpacityEffect*>(w->graphicsEffect());
        if (!eff) {
            eff = new QGraphicsOpacityEffect(w);
            w->setGraphicsEffect(eff);
            eff->setOpacity(1.0);
        }
        const QPoint origin = w->pos();
        QPointer<QWidget> guard(w);
        QPointer<QGraphicsOpacityEffect> effGuard(eff);
        QTimer::singleShot(i * staggerDelay, w,
            [guard, effGuard, origin, duration]() {
                if (!guard || !effGuard) return;
                auto* fade = new QPropertyAnimation(effGuard.data(), "opacity");
                fade->setDuration(duration);
                fade->setStartValue(effGuard->opacity());
                fade->setEndValue(0.0);
                fade->setEasingCurve(QEasingCurve::OutCubic);
                auto* slide = new QPropertyAnimation(guard.data(), "pos");
                slide->setDuration(duration);
                slide->setStartValue(guard->pos());
                slide->setEndValue(QPoint(origin.x(), origin.y() - 10));
                slide->setEasingCurve(QEasingCurve::OutCubic);
                auto* group = new QParallelAnimationGroup(guard.data());
                group->addAnimation(fade);
                group->addAnimation(slide);
                group->start(QAbstractAnimation::DeleteWhenStopped);
            });
    }
}

//------------------------------------------------------------------------------
// ActivityIndicator — iOS 圆形进度指示器 (补充六)
//------------------------------------------------------------------------------
ActivityIndicator::ActivityIndicator(QWidget* parent, int size)
    : QWidget(parent)
{
    setFixedSize(size, size);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
        angle_ = (angle_ + 30) % 360;
        update();
    });
    hide();
}

void ActivityIndicator::start() {
    if (!timer_->isActive()) {
        timer_->start(83);   // ~12 fps, iOS 风格步进
    }
    QWidget::show();
    raise();
}

void ActivityIndicator::stop() {
    timer_->stop();
    hide();
}

bool ActivityIndicator::isRunning() const {
    return timer_ && timer_->isActive();
}

void ActivityIndicator::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.translate(width() / 2.0, height() / 2.0);

    constexpr int numLines = 12;
    const double lineLen = width() * 0.2;
    const double lineW   = qMax(1.5, width() * 0.08);
    const double radius  = width() * 0.3;

    for (int i = 0; i < numLines; ++i) {
        p.save();
        const double angle = i * (360.0 / numLines);
        p.rotate(angle);
        // 距当前 angle_ 越远越淡, 形成尾迹
        int dist = (i * (360 / numLines) - angle_ + 360) % 360;
        double opacity = 1.0 - (dist / 360.0);
        opacity = qMax(0.15, opacity);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(142, 142, 147,
                          static_cast<int>(255 * opacity)));
        p.drawRoundedRect(QRectF(-lineW / 2.0, -radius - lineLen,
                                 lineW, lineLen),
                          lineW / 2.0, lineW / 2.0);
        p.restore();
    }
}

//------------------------------------------------------------------------------
// SkeletonBlock — 骨架屏占位 (补充七)
//------------------------------------------------------------------------------
SkeletonBlock::SkeletonBlock(QWidget* parent, int height)
    : QWidget(parent)
{
    setFixedHeight(height);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    auto* anim = new QPropertyAnimation(this, "shimmerPosition", this);
    anim->setDuration(1500);
    anim->setStartValue(-1.0);
    anim->setEndValue(2.0);
    anim->setLoopCount(-1);
    anim->setEasingCurve(QEasingCurve::Linear);
    anim->start();
}

void SkeletonBlock::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // 灰色底 (iOS systemGray5)
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(229, 229, 234));
    p.drawRoundedRect(rect(), 4, 4);

    // 高光扫过: 从 shimmer_pos_ 处打一个白色软渐变, 限制在圆角内
    QLinearGradient shimmer(width() * shimmer_pos_, 0,
                            width() * (shimmer_pos_ + 0.5), 0);
    shimmer.setColorAt(0.0, Qt::transparent);
    shimmer.setColorAt(0.5, QColor(255, 255, 255, 80));
    shimmer.setColorAt(1.0, Qt::transparent);

    QPainterPath clip;
    clip.addRoundedRect(QRectF(rect()), 4, 4);
    p.setClipPath(clip);
    p.fillRect(rect(), shimmer);
}

//------------------------------------------------------------------------------
// IOSContextMenu — iOS 风长按上下文菜单 (补充八)
//------------------------------------------------------------------------------
void IOSContextMenu::show(QWidget* anchor,
                          const QPoint& globalPos,
                          const QList<MenuItem>& items) {
    if (items.isEmpty()) return;
    Q_UNUSED(anchor);   // Popup 顶层窗口不需要 parent 参与显示
    auto* menu = new IOSContextMenu(globalPos, items);
    menu->animateIn();
}

IOSContextMenu::IOSContextMenu(const QPoint& globalPos,
                               const QList<MenuItem>& items)
    : QWidget(nullptr,
              Qt::Popup | Qt::FramelessWindowHint | Qt::NoDropShadowWindowHint),
      items_(items)
{
    // 透明顶层弹窗: 失焦自动关闭, 不污染父级布局
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_DeleteOnClose);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    const int menuH = items_.size() * kItemH;
    // 整个画布留 kPadding 边给阴影
    setFixedSize(kMenuW + kPadding * 2, menuH + kPadding * 2);

    // 把菜单几何中心定位到 globalPos 的下方 10px (避开手指/光标)
    QPoint topLeft(globalPos.x() - width() / 2,
                   globalPos.y() - 10);

    // 屏幕边界裁剪 (Qt5 用 QGuiApplication::screenAt)
    if (auto* screen = QGuiApplication::screenAt(globalPos)) {
        const QRect avail = screen->availableGeometry();
        topLeft.setX(qBound(avail.left(),  topLeft.x(),
                            avail.right()  - width()));
        topLeft.setY(qBound(avail.top(),   topLeft.y(),
                            avail.bottom() - height()));
    }
    move(topLeft);

    QWidget::show();
    raise();
    activateWindow();
    setFocus();
}

void IOSContextMenu::animateIn() {
    SmoothUI::springAnimate(this, "menuScale",   1.0, 300, 1.1);
    SmoothUI::smoothAnimate(this, "menuOpacity", 1.0, 150);
}

void IOSContextMenu::animateOut() {
    if (closing_) return;
    closing_ = true;
    SmoothUI::smoothAnimate(this, "menuOpacity", 0.0, 200);
    SmoothUI::smoothAnimate(this, "menuScale",   0.95, 200);
    QTimer::singleShot(220, this, &QWidget::close);
}

void IOSContextMenu::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setOpacity(menu_opacity_);

    // 以菜单中心为锚点缩放 (kPadding 边距 + kMenuW 内容)
    const QPointF center(width() / 2.0, kPadding + items_.size() * kItemH / 2.0);
    p.translate(center);
    p.scale(menu_scale_, menu_scale_);
    p.translate(-center);

    const QRect contentRect(kPadding, kPadding, kMenuW, items_.size() * kItemH);

    // 阴影 (走 ShadowUtils 的 popover 三层叠加)
    ShadowUtils::drawPopoverShadow(p, contentRect, 14);

    // 毛玻璃白底 (Squircle 形状)
    Squircle::draw(p, QRectF(contentRect), 14,
                   QBrush(QColor(255, 255, 255, 235)));

    // 用 Squircle 路径裁剪, hover 高亮和分割线都不溢出圆角
    QPainterPath clip = Squircle::path(QRectF(contentRect), 14);
    p.setClipPath(clip);

    p.setFont(SmoothUI::labelFont(12));
    for (int i = 0; i < items_.size(); ++i) {
        const QRect itemRect(contentRect.x(),
                             contentRect.y() + i * kItemH,
                             contentRect.width(), kItemH);

        // hover 高亮
        if (i == hover_index_) {
            p.fillRect(itemRect, QColor(0, 0, 0, 8));
        }

        // 标题
        const QColor textColor = items_[i].textColor.isValid()
                                 ? items_[i].textColor
                                 : Theme::textPrimary;
        p.setPen(textColor);
        p.drawText(itemRect.adjusted(20, 0, -20, 0),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   items_[i].title);

        // 分割线 (除最后一项外)
        if (i < items_.size() - 1) {
            p.setPen(QPen(QColor(0, 0, 0, 12), 0.5));
            p.drawLine(itemRect.left() + 20, itemRect.bottom(),
                       itemRect.right(),     itemRect.bottom());
        }
    }
}

void IOSContextMenu::mouseMoveEvent(QMouseEvent* e) {
    const int relY = e->pos().y() - kPadding;
    int idx = -1;
    if (relY >= 0 && relY < items_.size() * kItemH &&
        e->pos().x() >= kPadding && e->pos().x() < kPadding + kMenuW) {
        idx = relY / kItemH;
    }
    if (idx != hover_index_) {
        hover_index_ = idx;
        update();
    }
}

void IOSContextMenu::mouseReleaseEvent(QMouseEvent* e) {
    if (closing_) return;
    const int relY = e->pos().y() - kPadding;
    if (relY >= 0 && relY < items_.size() * kItemH &&
        e->pos().x() >= kPadding && e->pos().x() < kPadding + kMenuW) {
        const int idx = relY / kItemH;
        if (idx >= 0 && idx < items_.size() && items_[idx].action) {
            items_[idx].action();
        }
    }
    animateOut();
}

void IOSContextMenu::focusOutEvent(QFocusEvent*) {
    animateOut();
}

void IOSContextMenu::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Escape) {
        animateOut();
        return;
    }
    QWidget::keyPressEvent(e);
}

//==============================================================================
// 自绘仪表 — CircularGauge 圆形仪表
//==============================================================================
CircularGauge::CircularGauge(const QString& label, const QString& unit, QWidget* parent)
    : QWidget(parent), label_(label), unit_(unit) {
    setMinimumSize(160, 150);
}

void CircularGauge::setValue(double v) {
    if (qFuzzyCompare(target_value_, v)) return;
    target_value_ = v;
    // 取消旧动画, 从当前绘制值 value_ 平滑过渡到新 target
    if (anim_) { anim_->stop(); anim_->deleteLater(); }
    anim_ = new QVariantAnimation(this);
    anim_->setStartValue(value_);
    anim_->setEndValue(v);
    anim_->setDuration(400);
    anim_->setEasingCurve(QEasingCurve::OutCubic);
    connect(anim_, &QVariantAnimation::valueChanged, this, [this](const QVariant& val) {
        value_ = val.toDouble();
        update();
    });
    anim_->start();
}

void CircularGauge::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // 1. 卡片背景 (特斯拉白 + 极浅边框)
    QRectF bgRect = rect().adjusted(1, 1, -1, -1);
    p.setBrush(Theme::bgBase);
    p.setPen(QPen(Theme::borderDefault, 1));
    p.drawRoundedRect(bgRect, Spacing::panelRadius, Spacing::panelRadius);

    // 2. 仪表弧区域 (留出底部 28px 给标签)
    int sz = qMin(width(), height() - 28);
    QRectF arcRect((width() - sz) / 2.0 + 12, 12, sz - 24, sz - 24);

    const int START_ANGLE = 225;
    const int SPAN_ANGLE  = -270;

    // 3. 轨道 (极浅灰, 特斯拉白风不用深色轨道)
    QPen trackPen(Theme::borderDefault, 10, Qt::SolidLine, Qt::RoundCap);
    p.setPen(trackPen);
    p.drawArc(arcRect, START_ANGLE * 16, SPAN_ANGLE * 16);

    // 4. 判定颜色 (蓝/黄/红, 来自 Theme 功能色)
    bool isDanger = low_is_danger_ ? (value_ <= danger_) : (value_ >= danger_);
    bool isWarn   = low_is_danger_ ? (value_ <= warn_)   : (value_ >= warn_);
    QColor progColor = isDanger ? Theme::colorError
                     : isWarn   ? Theme::colorWarn
                                : Theme::accentBlue;

    // 5. 进度弧
    double norm = qBound(0.0, (value_ - min_) / (max_ - min_), 1.0);
    double span = SPAN_ANGLE * norm;
    QPen progPen(progColor, 10, Qt::SolidLine, Qt::RoundCap);
    p.setPen(progPen);
    p.drawArc(arcRect, START_ANGLE * 16, (int)(span * 16));

    // 6. 中心数字 (DemiBold 而非 Bold, 特斯拉偏细)
    p.setPen(Theme::textPrimary);
    int numPx = qMax(18, sz / 6);
    QFont numFont = SmoothUI::valueFont(numPx);
    numFont.setPixelSize(numPx);
    p.setFont(numFont);
    QRectF numRect = arcRect;
    numRect.moveTop(numRect.top() - 4);
    p.drawText(numRect, Qt::AlignCenter, QString::number(value_, 'f', decimals_));

    // 7. 单位 (Light 字重)
    p.setPen(Theme::textMuted);
    QFont uf = SmoothUI::unitFont(qMax(8, numPx / 2 - 1));
    uf.setPixelSize(qMax(8, numPx / 2 - 1));
    p.setFont(uf);
    QRectF unitRect = arcRect;
    unitRect.moveTop(arcRect.center().y() + numPx / 2 - 2);
    p.drawText(unitRect, Qt::AlignHCenter | Qt::AlignTop, unit_);

    // 8. 底部标签
    p.setPen(Theme::textSecondary);
    p.setFont(SmoothUI::labelFont(10));
    QRectF lblRect(0, height() - 24, width(), 22);
    p.drawText(lblRect, Qt::AlignCenter, label_);
}

//==============================================================================
// 自绘仪表 — LevelBar 水平仪条
//==============================================================================
LevelBar::LevelBar(const QString& label, double range, QWidget* parent)
    : QWidget(parent), label_(label), range_(range) {
    setMinimumSize(180, 72);
    setMaximumHeight(80);
}

void LevelBar::setValue(double v) {
    if (qFuzzyCompare(target_value_, v)) return;
    target_value_ = v;
    if (anim_) { anim_->stop(); anim_->deleteLater(); }
    anim_ = new QVariantAnimation(this);
    anim_->setStartValue(value_);
    anim_->setEndValue(v);
    anim_->setDuration(350);
    anim_->setEasingCurve(QEasingCurve::OutCubic);
    connect(anim_, &QVariantAnimation::valueChanged, this, [this](const QVariant& val) {
        value_ = val.toDouble();
        update();
    });
    anim_->start();
}

void LevelBar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // 卡片背景 (特斯拉白)
    QRectF bgRect = rect().adjusted(1, 1, -1, -1);
    p.setBrush(Theme::bgBase);
    p.setPen(QPen(Theme::borderDefault, 1));
    p.drawRoundedRect(bgRect, Spacing::inputRadius, Spacing::inputRadius);

    int w = width(), h = height();

    // 标签 (左上)
    p.setPen(Theme::textSecondary);
    p.setFont(SmoothUI::labelFont(9));
    p.drawText(QRect(12, 6, w - 24, 16), Qt::AlignLeft | Qt::AlignVCenter, label_);

    // 值 (右上, DemiBold 等宽)
    p.setPen(Theme::textPrimary);
    p.setFont(SmoothUI::valueFont(14));
    QString valStr = QString("%1\u00B0").arg(value_, 0, 'f', 1);
    p.drawText(QRect(12, 4, w - 24, 24), Qt::AlignRight | Qt::AlignVCenter, valStr);

    // 水平条轨道 (极浅灰, 3px 细)
    int barL = 14, barR = w - 14;
    int barY = h - 24;
    int barH = 3;
    QRect trackRect(barL, barY, barR - barL, barH);
    p.setPen(Qt::NoPen);
    p.setBrush(Theme::sliderTrack);
    p.drawRoundedRect(trackRect, 1.5, 1.5);

    // 中心刻度线
    int cx = (barL + barR) / 2;
    p.setPen(QPen(Theme::textDisabled, 1));
    p.drawLine(cx, barY - 4, cx, barY + barH + 4);

    // 指针 (白圆 + 色描边, 特斯拉风)
    double norm = qBound(-1.0, value_ / range_, 1.0);
    int px = cx + static_cast<int>(norm * (barR - barL) / 2.0);
    QColor ptColor = (std::abs(value_) >= range_ * 0.8) ? Theme::colorError
                   : (std::abs(value_) >= range_ * 0.5) ? Theme::colorWarn
                                                       : Theme::accentBlue;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 18));
    p.drawEllipse(QPoint(px, barY + barH / 2 + 1), 7, 7);
    p.setBrush(Theme::bgBase);
    p.setPen(QPen(ptColor, 1.5));
    p.drawEllipse(QPoint(px, barY + barH / 2), 6, 6);

    // 刻度标签 (浅灰)
    p.setPen(Theme::textMuted);
    p.setFont(SmoothUI::unitFont(8));
    p.drawText(QRect(barL - 8, barY + barH + 4, 40, 12),
               Qt::AlignLeft, QString("-%1\u00B0").arg(range_, 0, 'f', 0));
    p.drawText(QRect(barR - 32, barY + barH + 4, 40, 12),
               Qt::AlignRight, QString("+%1\u00B0").arg(range_, 0, 'f', 0));
}

//==============================================================================
// 自绘仪表 — StatusPill 状态胶囊
//==============================================================================
void StatusPill::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QRect r = rect().adjusted(1, 1, -1, -1);

    // 卡片背景 (特斯拉白)
    p.setBrush(Theme::bgBase);
    p.setPen(QPen(Theme::borderDefault, 1));
    p.drawRoundedRect(r, Spacing::inputRadius, Spacing::inputRadius);

    // 左侧状态圆点 (规则九: 纯色小圆点, 无呼吸动画, 弱化光晕)
    QColor lightColor;
    switch (state_) {
        case PILL_ON:     lightColor = Theme::colorOk;       break;
        case PILL_NORMAL: lightColor = Theme::accentBlue;    break;
        case PILL_WARN:   lightColor = Theme::colorWarn;     break;
        case PILL_DANGER: lightColor = Theme::colorError;    break;
        default:          lightColor = Theme::textDisabled;  break;
    }
    int cy = r.center().y();
    int cx = r.left() + 16;
    // 极淡色晕 (特斯拉风, 静态, 无呼吸)
    QColor halo = lightColor; halo.setAlpha(38);
    p.setPen(Qt::NoPen);
    p.setBrush(halo);
    p.drawEllipse(QPoint(cx, cy), 8, 8);
    // 实心圆点
    p.setBrush(lightColor);
    p.drawEllipse(QPoint(cx, cy), 4, 4);

    // 标签 (上)
    p.setPen(Theme::textSecondary);
    p.setFont(SmoothUI::labelFont(8));
    p.drawText(QRect(cx + 12, r.top() + 6, r.width() - (cx + 16), 14),
               Qt::AlignLeft | Qt::AlignVCenter, label_);

    // 值 (下, DemiBold 等宽)
    p.setPen(Theme::textPrimary);
    p.setFont(SmoothUI::valueFont(11));
    p.drawText(QRect(cx + 12, r.top() + 20, r.width() - (cx + 16), r.height() - 24),
               Qt::AlignLeft | Qt::AlignVCenter, value_);
}

//==============================================================================
ManualControlWidget::ManualControlWidget(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent), nh_(nh), rosbag_process_(nullptr)
{
    setWindowTitle("\u63A8\u571F\u673A\u65E0\u4EBA\u9A7E\u9A76\u63A7\u5236\u53F0");
    setWindowIcon(QIcon(loadPngPixmap(":/img/app_icon.png")));
    resize(1920, 1080);

    diag_elapsed_.start();
    setupUI();
    setupPublishers();
    setupSubscribers();
    setupConnections();

    direct_timer_ = new QTimer(this);
    connect(direct_timer_, &QTimer::timeout, this, &ManualControlWidget::onDirectTimerTick);
    direct_timer_->start(20);

    node_timer_ = new QTimer(this);
    connect(node_timer_, &QTimer::timeout, this, &ManualControlWidget::onNodeTimerTick);
    // 默认不启动, 切到Tab2时才启
    node_timer_->stop();

    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &ManualControlWidget::onStatusTimerTick);
    status_timer_->start(200);

    // 安全: Tab切换时停止非当前Tab的控制定时器
    connect(tabs_, &QTabWidget::currentChanged, this, &ManualControlWidget::onTabChanged);

    // 防止鼠标滚轮误改参数: 所有 SpinBox 和 ComboBox 必须先点击聚焦才能滚轮调节
    // (无焦点时 eventFilter 会把 wheel 转发给父 QScrollArea, 页面仍可正常滚动)
    for (auto* sb : findChildren<QDoubleSpinBox*>())
        sb->installEventFilter(this);
    for (auto* cb : findChildren<QComboBox*>())
        cb->installEventFilter(this);

    // 统一加大 QScrollArea 单步滚动步长, 让滚轮滑动更"跟手"
    // (默认 singleStep=1 非常碎, 大面板上体感跳格)
    for (auto* sa : findChildren<QScrollArea*>()) {
        if (auto* vsb = sa->verticalScrollBar())   vsb->setSingleStep(24);
        if (auto* hsb = sa->horizontalScrollBar()) hsb->setSingleStep(24);
    }

    // 启动自动加载: 如果默认配置文件存在, 自动恢复上次参数
    QString default_cfg = "./params/default.txt";
    if (QFile::exists(default_cfg)) {
        loadConfig(default_cfg);
        LOG_INFO("Auto-loaded config from %s", default_cfg.toStdString().c_str());
    }

    LOG_INFO("Control console initialized (1280x800)");
}

bool ManualControlWidget::eventFilter(QObject* obj, QEvent* event) {
    if (event->type() == QEvent::Wheel) {
        auto* w = qobject_cast<QWidget*>(obj);
        if (w && !w->hasFocus()) {
            // 无焦点时: 不吃掉滚轮事件, 而是转发给最近的 QScrollArea,
            // 让页面继续丝滑滚动 (之前 return true 会让外层收不到 wheel)
            for (QWidget* p = w->parentWidget(); p; p = p->parentWidget()) {
                if (auto* sa = qobject_cast<QScrollArea*>(p)) {
                    QApplication::sendEvent(sa->verticalScrollBar(), event);
                    return true;
                }
            }
            event->ignore();
            return true;
        }
    }
    // 联合施工动画: 按可用区域等比缩放, 保持原始比例
    if (obj == animation_widget_ && event->type() == QEvent::Resize) {
        if (animation_movie_) {
            QSize orig(648, 551);  // daolu.gif 原始分辨率
            QSize avail = animation_widget_->size();
            avail.setWidth (std::max(50, avail.width()  - 16));
            avail.setHeight(std::max(50, avail.height() - 16));
            QSize target = orig.scaled(avail, Qt::KeepAspectRatio);
            animation_movie_->setScaledSize(target);
        }
    }
    // 折叠按钮点击
    if (obj->objectName() == "collapseBtn" && event->type() == QEvent::MouseButtonPress) {
        toggleDrawer();
        return true;
    }
    // 拖拽手柄: 按下/移动/释放
    if (obj->objectName() == "drawerHandle") {
        if (event->type() == QEvent::MouseButtonPress) {
            auto* me = static_cast<QMouseEvent*>(event);
            drawer_dragging_ = true;
            drawer_drag_start_y_ = me->globalY();
            drawer_drag_start_h_ = tabs_->height();
            return true;
        }
        if (event->type() == QEvent::MouseMove && drawer_dragging_) {
            auto* me = static_cast<QMouseEvent*>(event);
            int delta = drawer_drag_start_y_ - me->globalY();  // 往上拖=正值=增大高度
            int tab_bar_h = tabs_->tabBar()->sizeHint().height() + 4;
            int max_h = this->height() - 200;  // 地图至少保留200px
            int new_h = std::max(tab_bar_h + 50, std::min(max_h, drawer_drag_start_h_ + delta));
            tabs_->setFixedHeight(new_h);
            drawer_height_ = new_h;
            return true;
        }
        if (event->type() == QEvent::MouseButtonRelease) {
            drawer_dragging_ = false;
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

ManualControlWidget::~ManualControlWidget() {
    // 立即停止所有定时器
    direct_timer_->stop();
    node_timer_->stop();
    status_timer_->stop();

    // 快速杀掉录制进程
    if (rosbag_process_ && rosbag_process_->state() != QProcess::NotRunning) {
        rosbag_process_->kill();  // kill 比 terminate 快
        rosbag_process_->waitForFinished(500);
    }
}

QPushButton* ManualControlWidget::createHoldButton(const QString& text, const QString& style) {
    auto* b = new QPushButton(text, this);
    b->setMinimumSize(90, 50);
    b->setFont(QFont("", 12, QFont::Bold));
    if (!style.isEmpty()) b->setStyleSheet(style);
    return b;
}

QDoubleSpinBox* ManualControlWidget::makeSpin(double min, double max, double val, double step, int dec) {
    // 工厂改为返回 SmartSpinBox (继承 QDoubleSpinBox, API 完全兼容):
    //   - 底部彩色范围条, 可视化当前值在范围内的位置
    //   - 滚轮修饰键: Shift+滚轮 ×10 粗调 / Alt+滚轮 ÷10 细调
    auto* s = new SmartSpinBox(this);
    s->setRange(min, max); s->setValue(val); s->setSingleStep(step); s->setDecimals(dec);
    return s;
}

QGroupBox* ManualControlWidget::makeGroup(const QString& title) {
    // iOS 融合风卡片 (融合风规则三/六): 纯 QSS 实现, 不用 QGraphicsEffect.
    //   (之前尝试加 QGraphicsDropShadowEffect, 会让子控件自绘 paintEvent 产生
    //    "Painter not active" 警告刷屏, 因为 effect 会改变 widget 渲染路径.
    //    改为用双层浅色 border 模拟 iOS 卡片边缘, 视觉上仍然有层次感.)
    //   - 14px iOS 14+ 标准圆角
    //   - 双层边框: 外层 1px alpha 8% + 内层细微 1px alpha 4%, 模拟阴影
    //   - 标题字重 600 + 微紧缩字间距 (iOS headline 特征)
    auto* g = new QGroupBox(title, this);
    g->setStyleSheet(QStringLiteral(
        "QGroupBox{"
        "background:%1;"
        "border:1px solid rgba(0,0,0,0.08);"
        "border-radius:%2px;"
        "margin-top:20px;"
        "padding:%3px %3px %3px %3px;"
        "padding-top:%4px;"
        "font-size:12px;font-weight:500;"
        "color:%5;}"
        "QGroupBox::title{"
        "subcontrol-origin:margin;"
        "subcontrol-position:top left;"
        "padding:2px 14px 6px 14px;"
        "color:%6;"
        "font-weight:600;"
        "font-size:13px;"
        "letter-spacing:-0.2px;}"
    ).arg(Theme::hex(Theme::bgSurface),
          QString::number(Spacing::panelRadius),
          QString::number(Spacing::panelPadding),
          QString::number(Spacing::panelPadding + 10),
          Theme::hex(Theme::textSecondary),
          Theme::hex(Theme::textPrimary)));
    return g;
}

ParamLabel* ManualControlWidget::makeParamLabel(const QString& text,
                                                const QString& tooltip,
                                                const QString& detail)
{
    return new ParamLabel(text, tooltip, detail, this);
}

//==============================================================================
// UI 主布局 — 1920×1080 深色科技风
// 左侧栏 | 右侧上方:地图(常驻) / 右侧下方:功能Tab
//==============================================================================
void ManualControlWidget::setupUI() {
    // 特斯拉白色风格全局基线 (规则四). 所有控件:
    //  - 纯白主背景, 极浅灰 (#E0E0E5) 边框几乎隐形
    //  - 聚焦边框 #007AFF 唯一的蓝
    //  - 滚动条 6px 极细浅灰
    //  - ToolTip 深色反转, 圆角 6px
    // 输入框字体用 SF Mono 等宽, 文字用 SF Pro Display + PingFang SC + 微软雅黑 fallback
    setStyleSheet(QString(R"(
        QWidget {
            background: #F5F5F7;
            color: #000000;
            font-family: "SF Pro Text","SF Pro Display","Helvetica Neue","PingFang SC","Microsoft YaHei UI",sans-serif;
            font-size: 13px;
        }
        /* === TabWidget === */
        QTabWidget::pane {
            border: 1px solid #E0E0E5;
            background: #FFFFFF;
            border-radius: 12px;
            margin-top: -1px;
        }
        QTabBar::tab {
            background: transparent;
            color: #8E8E93;
            padding: 12px 22px;
            border: none;
            border-bottom: 2px solid transparent;
            min-width: 90px;
            font-weight: 500;
            font-size: 13px;
            margin-right: 4px;
        }
        QTabBar::tab:selected {
            color: #000000;
            border-bottom: 2px solid #007AFF;
        }
        QTabBar::tab:hover:!selected {
            color: #2C2C2E;
        }
        QLabel { color: #000000; background: transparent; }
        /* === 输入控件 === */
        QDoubleSpinBox, QSpinBox, QLineEdit {
            background: #FFFFFF;
            color: #000000;
            border: 1px solid #E0E0E5;
            border-radius: 8px;
            padding: 8px 12px;
            font-family: "SF Mono","JetBrains Mono","Consolas",monospace;
            font-size: 13px;
            selection-background-color: #CCE0FF;
            selection-color: #0055B3;
        }
        QDoubleSpinBox:hover, QSpinBox:hover, QLineEdit:hover {
            border-color: #D1D1D6;
        }
        QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {
            border-color: #007AFF;
        }
        QDoubleSpinBox:disabled, QSpinBox:disabled, QLineEdit:disabled {
            background: #F9FAFB;
            color: #D1D1D6;
            border-color: #F3F4F6;
        }
        QTextEdit {
            background: #FFFFFF;
            color: #2C2C2E;
            border: 1px solid #E0E0E5;
            border-radius: 8px;
            padding: 8px 10px;
            font-family: "SF Mono","JetBrains Mono","Consolas",monospace;
            selection-background-color: #CCE0FF;
            selection-color: #0055B3;
        }
        QTextEdit:focus { border-color: #007AFF; }
        /* === CheckBox / RadioButton === */
        QCheckBox, QRadioButton { color: #000000; spacing: 8px; }
        QCheckBox::indicator, QRadioButton::indicator { width: 16px; height: 16px; }
        /* === ComboBox === */
        QComboBox {
            background: #FFFFFF;
            color: #000000;
            border: 1px solid #E0E0E5;
            border-radius: 8px;
            padding: 8px 12px;
            padding-right: 28px;
        }
        QComboBox:hover { border-color: #D1D1D6; }
        QComboBox:focus { border-color: #007AFF; }
        QComboBox::drop-down { border: none; width: 24px; }
        QComboBox QAbstractItemView {
            background: #FFFFFF;
            border: 1px solid #E0E0E5;
            border-radius: 8px;
            selection-background-color: #E6F0FF;
            selection-color: #0055B3;
            outline: none;
            padding: 4px;
        }
        /* === Slider (特斯拉白轨道 + 纯蓝填充) === */
        QSlider::groove:horizontal {
            background: #E5E7EB;
            height: 3px;
            border-radius: 1px;
        }
        QSlider::sub-page:horizontal { background: #007AFF; border-radius: 1px; }
        QSlider::handle:horizontal {
            background: #FFFFFF;
            border: 1.5px solid #007AFF;
            width: 16px; height: 16px;
            margin: -7px 0;
            border-radius: 8px;
        }
        QSlider::handle:horizontal:hover { border-color: #0062D4; }
        /* === ScrollArea + 极细滚动条 === */
        QScrollArea { border: none; background: #FFFFFF; }
        QScrollBar:vertical {
            background: transparent;
            width: 6px;
            margin: 4px 1px;
        }
        QScrollBar::handle:vertical {
            background: #D1D1D6;
            border-radius: 3px;
            min-height: 30px;
        }
        QScrollBar::handle:vertical:hover { background: #9CA3AF; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
        QScrollBar:horizontal {
            background: transparent;
            height: 6px;
            margin: 1px 4px;
        }
        QScrollBar::handle:horizontal {
            background: #D1D1D6;
            border-radius: 3px;
            min-width: 30px;
        }
        QScrollBar::handle:horizontal:hover { background: #9CA3AF; }
        QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
        QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }
        /* === ToolTip 深色反转 === */
        QToolTip {
            background: #000000;
            color: #F9FAFB;
            border: none;
            border-radius: 6px;
            padding: 6px 12px;
            font-size: 12px;
        }
    )"));

    auto* mainH = new QHBoxLayout(this);
    mainH->setSpacing(0);
    mainH->setContentsMargins(0,0,0,0);

    // 左侧状态栏
    mainH->addWidget(createSidebar());

    // 右侧: 地图占满 + Tab 固定在底部 (抽屉式)
    auto* rightW = new QWidget(this);
    auto* rightV = new QVBoxLayout(rightW);
    rightV->setSpacing(0);
    rightV->setContentsMargins(4,4,4,4);

    // 地图区 (占满, stretch=1) — 栅格地图 ↔ RViz 感知视图切换
    main_display_stack_ = new QStackedWidget(this);
    main_display_stack_->setMinimumHeight(200);

    // Page 0: 栅格地图
    map_widget_ = new GridMapWidget(nh_, this);
    map_widget_->setStyleSheet("background:white;border:1px solid #E0E0E5;border-radius:12px;");
    main_display_stack_->addWidget(map_widget_);

    // Page 1: RViz 感知视图
    rviz_widget_ = new RvizPerceptionWidget(this);
    rviz_widget_->setStyleSheet("background:white;border:1px solid #E0E0E5;border-radius:12px;");
    main_display_stack_->addWidget(rviz_widget_);

    // Page 2: 联合施工动画 (默认页)
    animation_widget_ = new QLabel(this);
    animation_widget_->setAlignment(Qt::AlignCenter);
    animation_widget_->setStyleSheet(
        "QLabel{background:white;border:1px solid #E0E0E5;border-radius:12px;}");
    animation_widget_->setObjectName("animationView");
    animation_widget_->installEventFilter(this);  // 响应 resize 保持比例缩放
    animation_movie_ = new QMovie(":/img/daolu.gif");
    animation_widget_->setMovie(animation_movie_);
    animation_movie_->start();
    main_display_stack_->addWidget(animation_widget_);

    main_display_stack_->setCurrentIndex(2);  // 默认显示联合施工动画
    // 默认不显示 RViz — 延迟初始化, 完全不触发 rviz 启动开销

    // 切换按钮浮在右上角
    auto* viewSwitchBar = new QWidget(this);
    viewSwitchBar->setFixedHeight(32);
    viewSwitchBar->setStyleSheet("background:transparent;");
    auto* switchHL = new QHBoxLayout(viewSwitchBar);
    switchHL->setContentsMargins(0, 2, 8, 0);
    switchHL->setSpacing(2);
    switchHL->addStretch();

    const char* STY_SW_ON  = "QPushButton{background:#007AFF;color:white;border:1px solid #007AFF;"
                             "border-radius:4px;padding:3px 14px;font-size:11px;font-weight:600;}";
    const char* STY_SW_OFF = "QPushButton{background:#F9FAFB;color:#8E8E93;border:1px solid #D1D1D6;"
                             "border-radius:4px;padding:3px 14px;font-size:11px;font-weight:600;}"
                             "QPushButton:hover{background:#E0E0E5;}";

    btn_view_animation_ = new QPushButton("\U0001F6A7 \u8054\u5408\u65BD\u5DE5", viewSwitchBar);
    btn_view_animation_->setStyleSheet(STY_SW_ON);
    btn_view_gridmap_ = new QPushButton("\U0001F5FA \u6805\u683C\u5730\u56FE", viewSwitchBar);
    btn_view_gridmap_->setStyleSheet(STY_SW_OFF);
    btn_view_rviz_ = new QPushButton("\U0001F441 \u611F\u77E5\u89C6\u56FE", viewSwitchBar);
    btn_view_rviz_->setStyleSheet(STY_SW_OFF);

    switchHL->addWidget(btn_view_animation_);
    switchHL->addWidget(btn_view_gridmap_);
    switchHL->addWidget(btn_view_rviz_);

    // 统一的按钮高亮切换逻辑
    auto updateViewButtons = [this, STY_SW_ON, STY_SW_OFF](int idx) {
        btn_view_animation_->setStyleSheet(idx == 2 ? STY_SW_ON : STY_SW_OFF);
        btn_view_gridmap_ ->setStyleSheet(idx == 0 ? STY_SW_ON : STY_SW_OFF);
        btn_view_rviz_   ->setStyleSheet(idx == 1 ? STY_SW_ON : STY_SW_OFF);
    };

    connect(btn_view_animation_, &QPushButton::clicked, [this, updateViewButtons]{
        main_display_stack_->setCurrentIndex(2);
        updateViewButtons(2);
        if (animation_movie_) animation_movie_->setPaused(false);
        if (rviz_widget_) rviz_widget_->pauseRendering();
    });
    connect(btn_view_gridmap_, &QPushButton::clicked, [this, updateViewButtons]{
        main_display_stack_->setCurrentIndex(0);
        updateViewButtons(0);
        if (animation_movie_) animation_movie_->setPaused(true);
        if (rviz_widget_) rviz_widget_->pauseRendering();
    });
    connect(btn_view_rviz_, &QPushButton::clicked, [this, updateViewButtons]{
        main_display_stack_->setCurrentIndex(1);
        updateViewButtons(1);
        if (animation_movie_) animation_movie_->setPaused(true);
        if (rviz_widget_) {
            // 首次点击: 触发延迟初始化 (首次有 1-2 秒加载感, 之后直接恢复)
            rviz_widget_->ensureInitialized();
            rviz_widget_->resumeRendering();
        }
    });

    rightV->addWidget(viewSwitchBar, 0);
    rightV->addWidget(main_display_stack_, 1);

    // Tab 区域 (底部抽屉)
    // 拖拽手柄 (展开后可拖动调节高度)
    drawer_handle_ = new QWidget(this);
    drawer_handle_->setFixedHeight(8);
    drawer_handle_->setCursor(Qt::SplitVCursor);
    drawer_handle_->setStyleSheet(
        "QWidget{background:transparent;}"
        "QWidget:hover{background:#D1D1D6;border-radius:2px;}");
    drawer_handle_->setVisible(false);  // 收起时隐藏
    drawer_handle_->installEventFilter(this);
    drawer_handle_->setObjectName("drawerHandle");
    // 手柄中间画一个小横杠提示
    auto* handleBar = new QWidget(drawer_handle_);
    handleBar->setFixedSize(40, 4);
    handleBar->setStyleSheet("background:#D1D1D6;border-radius:2px;");
    auto* handleLayout = new QHBoxLayout(drawer_handle_);
    handleLayout->setContentsMargins(0,2,0,2);
    handleLayout->setAlignment(Qt::AlignCenter);
    handleLayout->addWidget(handleBar);
    rightV->addWidget(drawer_handle_, 0);

    tabs_ = new QTabWidget(this);
    tabs_->setFont(QFont("", 10, QFont::Bold));
    setupTab1_DirectCAN(tabs_);
    setupTab2_NodeTest(tabs_);
    setupTab3_PIDTuning(tabs_);
    setupTab4_VehicleStatus(tabs_);
    setupTab5_PlanningParams(tabs_);
    setupTab6_VehicleConfig(tabs_);
    setupTab7_DataRecord(tabs_);
    setupTab8_CommDiag(tabs_);
    setupTab9_DiagSnapshot(tabs_);

    // 把 Tab 的内容区(pane) 包在一个可动画高度的容器里
    // 方法: 隐藏 QTabWidget 自带的 pane, 用 maximumHeight 控制
    // 实际上直接对 tabs_ 整体做 fixedHeight 动画更简单:
    //   收起时高度 = tabBar高度 (~40px)
    //   展开时高度 = tabBar高度 + 内容高度
    tabs_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    int tab_bar_h = 44;  // Tab标签栏高度 (padding 12*2 + font ~20)
    tabs_->setFixedHeight(tab_bar_h);  // 默认收起

    // 折叠指示箭头 (corner widget)
    auto* collapseBtn = new QLabel("\u25B2", this);
    collapseBtn->setFixedSize(28, 28);
    collapseBtn->setAlignment(Qt::AlignCenter);
    collapseBtn->setStyleSheet(
        "QLabel{color:#8E8E93;font-size:11px;background:#E0E0E5;"
        "border:1px solid #D1D1D6;border-radius:6px;}"
        "QLabel:hover{background:#D1D1D6;color:#007AFF;}");
    collapseBtn->setCursor(Qt::PointingHandCursor);
    collapseBtn->setToolTip("\u5C55\u5F00/\u6536\u8D77 (F11)");
    collapseBtn->installEventFilter(this);
    collapseBtn->setObjectName("collapseBtn");
    tabs_->setCornerWidget(collapseBtn, Qt::TopRightCorner);

    rightV->addWidget(tabs_, 0);  // stretch=0, 不抢地图空间

    mainH->addWidget(rightW, 1);

    // 抽屉动画
    drawer_anim_ = new QPropertyAnimation(tabs_, "maximumHeight", this);
    drawer_anim_->setDuration(200);
    drawer_anim_->setEasingCurve(QEasingCurve::OutCubic);

    // Tab 标签点击 → 折叠/展开
    connect(tabs_->tabBar(), &QTabBar::tabBarClicked, this, &ManualControlWidget::onTabBarClicked);

    // 阶段五: TabBar 横向 swipe 切换 Tab (iOS 风手势, 不消费事件 — 点击切 Tab 仍正常)
    GestureHandler::install(tabs_->tabBar())
        ->setSwipeMinDistance(50)
        ->setOnSwipe([this](GestureHandler::Direction d) {
            if (d != GestureHandler::DirLeft && d != GestureHandler::DirRight) return;
            int next = tabs_->currentIndex() + (d == GestureHandler::DirLeft ? 1 : -1);
            if (next < 0 || next >= tabs_->count()) return;
            tabs_->setCurrentIndex(next);
            if (!drawer_open_) toggleDrawer();
        });

    // F11 快捷键: 抽屉展开/收起
    shortcut_toggle_tab_ = new QShortcut(QKeySequence(Qt::Key_F11), this);
    connect(shortcut_toggle_tab_, &QShortcut::activated, this, &ManualControlWidget::toggleDrawer);

    // Ctrl+1~9: 快速切 Tab (工业软件标配, 减少鼠标操作)
    //   切 Tab 的同时若抽屉是收起的, 自动展开, 一步到位
    for (int i = 0; i < 9; ++i) {
        auto* sc = new QShortcut(QKeySequence(QString("Ctrl+%1").arg(i + 1)), this);
        connect(sc, &QShortcut::activated, this, [this, i]{
            if (i < tabs_->count()) {
                tabs_->setCurrentIndex(i);
                if (!drawer_open_) toggleDrawer();
            }
        });
    }

    // Esc: 紧急停止 (物理反应最快的键, 操作员遇险第一反应)
    auto* sc_esc = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(sc_esc, &QShortcut::activated, this, &ManualControlWidget::emergencyStop);

    // Tab 切换淡入动画 (180ms OutQuad, 不影响原 onTabChanged 业务逻辑)
    connect(tabs_, &QTabWidget::currentChanged, this, [this](int idx) {
        QWidget* w = tabs_->widget(idx);
        if (!w) return;
        auto* eff = qobject_cast<QGraphicsOpacityEffect*>(w->graphicsEffect());
        if (!eff) {
            eff = new QGraphicsOpacityEffect(w);
            w->setGraphicsEffect(eff);
        }
        auto* a = new QPropertyAnimation(eff, "opacity", this);
        a->setDuration(180);
        a->setStartValue(0.0);
        a->setEndValue(1.0);
        a->setEasingCurve(QEasingCurve::OutQuad);
        a->start(QAbstractAnimation::DeleteWhenStopped);
    });
}

//==============================================================================
// 左侧状态栏
//==============================================================================
QWidget* ManualControlWidget::createSidebar() {
    auto* sidebar = new QWidget(this);
    sidebar->setFixedWidth(230);
    sidebar->setStyleSheet(
        "QWidget#sidebar{background:white;border-right:1px solid #E0E0E5;}"
        "QLabel{color:#000000;padding:2px 8px;}");
    sidebar->setObjectName("sidebar");
    auto* vl = new QVBoxLayout(sidebar);
    vl->setSpacing(3);
    vl->setContentsMargins(10,10,10,10);

    // Logo (品牌徽章) + 公司名 + 产品名
    auto* logo = new QLabel(sidebar);
    QPixmap logoPix = loadPngPixmap(":/img/logo.png");
    logo->setPixmap(logoPix.scaled(96, 96, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    logo->setAlignment(Qt::AlignCenter);
    logo->setStyleSheet("padding:10px 0 4px 0;background:transparent;");
    vl->addWidget(logo);

    auto* brand = new QLabel("DOZER UNMANNED", sidebar);
    QFont brandFont("", 16, QFont::Black);
    brandFont.setLetterSpacing(QFont::AbsoluteSpacing, 3);
    brand->setFont(brandFont);
    brand->setAlignment(Qt::AlignCenter);
    brand->setStyleSheet("color:#007AFF;padding:0;background:transparent;");
    vl->addWidget(brand);

    auto* title = new QLabel("\u63A8\u571F\u673A\u63A7\u5236\u53F0", sidebar);
    title->setFont(QFont("", 11, QFont::Bold));
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("color:#8E8E93;padding:0 0 8px 0;background:transparent;");
    vl->addWidget(title);

    auto* sep = new QFrame(sidebar);
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color:#E0E0E5;");
    vl->addWidget(sep);

    // 侧边栏状态项: [彩色圆点] [图标+标题小字 / 值大字]
    //   - 圆点颜色反映状态 (灰=未知/off, 蓝=正常, 绿=活跃, 橙=警告, 红=危险)
    //   - setVal 刷新时会同步改圆点颜色, 通过 property "sidebar_dot" 定位
    //   - 返回值 label 签名保持兼容 onStatusTimerTick 原有调用
    auto addItem = [&](const QString& label, const QString& icon) -> QLabel* {
        auto* row = new QWidget(sidebar);
        row->setStyleSheet("QWidget{background:transparent;}");
        auto* rowH = new QHBoxLayout(row);
        rowH->setContentsMargins(8, 5, 8, 5);
        rowH->setSpacing(10);

        // 彩色圆点 (SideDot 自绘 + 颜色动画过渡, 取代 QLabel+setStyleSheet 瞬时切色)
        auto* dot = new SideDot(row);

        // 右列: 标题小字 + 值大字
        auto* col = new QWidget(row);
        col->setStyleSheet("QWidget{background:transparent;}");
        auto* colV = new QVBoxLayout(col);
        colV->setContentsMargins(0, 0, 0, 0);
        colV->setSpacing(0);

        auto* t = new QLabel(icon + " " + label, col);
        t->setFont(QFont("", 8));
        t->setStyleSheet("color:#AEAEB2;background:transparent;padding:0;");
        colV->addWidget(t);

        auto* v = new QLabel("--", col);
        v->setFont(QFont("", 12, QFont::Bold));
        v->setStyleSheet("color:#000000;background:transparent;padding:0;");
        colV->addWidget(v);

        rowH->addWidget(dot, 0, Qt::AlignVCenter);
        rowH->addWidget(col, 1);
        vl->addWidget(row);

        // 圆点挂到 value label 的动态 property 上, 供 setVal 同步色彩
        v->setProperty("sidebar_dot", QVariant::fromValue<QObject*>(dot));
        return v;
    };

    side_main_switch_   = addItem("\u4E3B\u5F00\u5173",   "\u23FB");
    side_state_machine_ = addItem("\u6267\u884C\u5668",   "\u2699");
    side_walk_state_    = addItem("\u884C\u8D70\u72B6\u6001", "\U0001F69C");
    side_heading_       = addItem("\u822A\u5411",       "\U0001F9ED");
    side_vehicle_speed_ = addItem("\u8F66\u901F",       "\u23F1");
    side_engine_rpm_    = addItem("\u53D1\u52A8\u673A",   "\u26A1");
    side_blade_height_  = addItem("\u94F2\u5200\u9AD8\u5EA6", "\u26CF");
    side_rtk_status_    = addItem("RTK\u5B9A\u4F4D",    "\U0001F4E1");
    side_can_status_    = addItem("CAN\u901A\u4FE1",    "\U0001F50C");

    // ── 感知风险状态灯 ──
    auto* riskSep = new QFrame(sidebar);
    riskSep->setFrameShape(QFrame::HLine); riskSep->setStyleSheet("background:#E0E0E5;"); riskSep->setFixedHeight(1);
    vl->addWidget(riskSep);
    auto* riskTitle = new QLabel("\U0001F6A8 \u611F\u77E5\u98CE\u9669", sidebar);
    riskTitle->setStyleSheet("color:#8E8E93;font-size:11px;font-weight:bold;padding:2px 8px;margin-top:3px;");
    vl->addWidget(riskTitle);

    // 十字布局: 前/左右/后, 模拟车辆俯视方位
    auto* riskGrid = new QGridLayout();
    riskGrid->setSpacing(3);
    riskGrid->setContentsMargins(8, 0, 8, 0);

    auto makeRiskLight = [&](const QString& label) -> QLabel* {
        auto* lbl = new QLabel(label, sidebar);
        lbl->setFixedSize(50, 28);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setFont(QFont("", 9, QFont::Bold));
        lbl->setStyleSheet(
            "background:#AEAEB2;color:white;border-radius:6px;");
        return lbl;
    };

    side_risk_front_ = makeRiskLight("\u524D");
    side_risk_back_  = makeRiskLight("\u540E");
    side_risk_left_  = makeRiskLight("\u5DE6");
    side_risk_right_ = makeRiskLight("\u53F3");

    // 车身占位 (推土机俯视图)
    auto* carIcon = new QLabel(sidebar);
    carIcon->setFixedSize(50, 28);
    carIcon->setAlignment(Qt::AlignCenter);
    QPixmap carPix = loadPngPixmap(":/img/bulldozer_top.png");
    carIcon->setPixmap(carPix.scaled(44, 26, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    carIcon->setStyleSheet("background:transparent;");

    //     前
    //  左 车 右
    //     后
    riskGrid->addWidget(side_risk_front_, 0, 1, Qt::AlignCenter);
    riskGrid->addWidget(side_risk_left_,  1, 0, Qt::AlignCenter);
    riskGrid->addWidget(carIcon,          1, 1, Qt::AlignCenter);
    riskGrid->addWidget(side_risk_right_, 1, 2, Qt::AlignCenter);
    riskGrid->addWidget(side_risk_back_,  2, 1, Qt::AlignCenter);
    vl->addLayout(riskGrid);

    // 当前配置文件
    auto* cfgSep = new QFrame(sidebar);
    cfgSep->setFrameShape(QFrame::HLine); cfgSep->setStyleSheet("background:#E0E0E5;"); cfgSep->setFixedHeight(1);
    vl->addWidget(cfgSep);
    auto* cfgTitle = new QLabel("\U0001F4C4 \u914D\u7F6E\u6587\u4EF6", sidebar);
    cfgTitle->setStyleSheet("color:#8E8E93;font-size:11px;padding:2px 8px;");
    vl->addWidget(cfgTitle);
    side_config_path_ = new QLabel("default.txt", sidebar);
    side_config_path_->setWordWrap(true);
    side_config_path_->setStyleSheet("color:#007AFF;font-size:10px;padding:2px 8px;font-family:Monospace;");
    vl->addWidget(side_config_path_);

    vl->addStretch();

    // 全部停止 / 恢复操作 — 开关按钮
    // iOS 急停按钮 — Danger (红) 常态, 停止后切 Success (绿) 恢复态.
    // 视觉由 IOSButton paintEvent 自绘, 不用 setStyleSheet.
    btn_emergency_stop_ = new IOSButton("\U0001F6D1  \u7D27\u6025\u505C\u6B62", IOSButton::Danger, sidebar);
    btn_emergency_stop_->setMinimumHeight(55);
    btn_emergency_stop_->setFont(QFont("", 14, QFont::Bold));
    connect(btn_emergency_stop_, &QPushButton::clicked, this, &ManualControlWidget::emergencyStop);
    vl->addWidget(btn_emergency_stop_);

    return sidebar;
}

//==============================================================================
// Tab1: 直接CAN控制
//==============================================================================
void ManualControlWidget::setupTab1_DirectCAN(QTabWidget* tabs) {
    auto* tab = new QWidget();
    auto* layout = new QVBoxLayout(tab);

    auto* wG = makeGroup("\u884C\u8D70\u63A7\u5236 (\u6309\u4F4F\u6709\u6548, \u677E\u5F00\u505C\u6B62)");
    auto* wL = new QGridLayout(wG);
    // 圆形方向按钮样式
    QString circStyle = "QPushButton{border-radius:30px;min-width:60px;min-height:60px;max-width:60px;max-height:60px;font-size:22px;font-weight:bold;}";
    // iOS 风阴影: 给圆形按钮加 QGraphicsDropShadowEffect "浮起"质感.
    // 每个按钮需独立 effect 实例 (effect 不能跨控件共享).
    auto attachCircleShadow = [](QPushButton* b) {
        auto* shadow = new QGraphicsDropShadowEffect(b);
        shadow->setBlurRadius(16);
        shadow->setColor(QColor(0, 0, 0, 55));
        shadow->setOffset(0, 3);
        b->setGraphicsEffect(shadow);
    };
    btn_forward_  = new QPushButton("\u25B2", this); btn_forward_->setStyleSheet(circStyle + "QPushButton{background:#34C759;color:white;}QPushButton:pressed{background:#2FA350;}"); attachCircleShadow(btn_forward_);
    btn_backward_ = new QPushButton("\u25BC", this); btn_backward_->setStyleSheet(circStyle + "QPushButton{background:#FF9500;color:white;}QPushButton:pressed{background:#C07300;}"); attachCircleShadow(btn_backward_);
    btn_left_     = new QPushButton("\u25C0", this); btn_left_->setStyleSheet(circStyle + "QPushButton{background:#007AFF;color:white;}QPushButton:pressed{background:#0055B3;}"); attachCircleShadow(btn_left_);
    btn_right_    = new QPushButton("\u25B6", this); btn_right_->setStyleSheet(circStyle + "QPushButton{background:#007AFF;color:white;}QPushButton:pressed{background:#0055B3;}"); attachCircleShadow(btn_right_);
    // 中间车辆图标 (推土机俯视图, 十字方向盘的视觉锚点)
    auto* centerIcon = new QLabel(this);
    centerIcon->setAlignment(Qt::AlignCenter);
    centerIcon->setStyleSheet("background:#F9FAFB;border-radius:30px;min-width:60px;min-height:60px;max-width:60px;max-height:60px;");
    QPixmap centerPix = loadPngPixmap(":/img/bulldozer_top.png");
    centerIcon->setPixmap(centerPix.scaled(48, 48, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    wL->addWidget(btn_forward_, 0,1, Qt::AlignCenter);
    wL->addWidget(btn_left_, 1,0, Qt::AlignCenter);
    wL->addWidget(centerIcon, 1,1, Qt::AlignCenter);
    wL->addWidget(btn_right_, 1,2, Qt::AlignCenter);
    wL->addWidget(btn_backward_, 2,1, Qt::AlignCenter);
    auto* stL = new QHBoxLayout();
    stL->addWidget(new QLabel("转向力度:"));
    slider_steering_ = new SmoothSlider(Qt::Horizontal); slider_steering_->setRange(50,255); slider_steering_->setValue(128);
    stL->addWidget(slider_steering_);
    label_steering_val_ = new QLabel("128"); stL->addWidget(label_steering_val_);
    wL->addLayout(stL, 3,0,1,3);
    auto* syL = new QHBoxLayout();
    btn_unlock_ = createHoldButton("\u89E3\u9501", STY_GRAY);
    btn_stop_ = createHoldButton("\u505C\u6B62\u9501\u5B9A", STY_RED); btn_stop_->setMinimumSize(130,50);
    btn_brake_ = new QPushButton("\u5239\u8F66", this);
    btn_brake_->setMinimumSize(80,50);
    btn_brake_->setStyleSheet("QPushButton{background:#FF3B30;color:white;border-radius:6px;font-weight:bold;}"
                               "QPushButton:pressed{background:#C02825;}");
    // 挡位方向显示 (N/F/R)
    label_gear_dir_display_ = new QLabel("N", this);
    label_gear_dir_display_->setFont(QFont("", 18, QFont::Bold));
    label_gear_dir_display_->setAlignment(Qt::AlignCenter);
    label_gear_dir_display_->setMinimumSize(50, 50);
    label_gear_dir_display_->setStyleSheet("background:white;color:#AEAEB2;border-radius:10px;border:2px solid #AEAEB2;");
    btn_gear_down_ = createHoldButton("\u964D\u6321 \u25BC", STY_GRAY);
    // 挡位大小显示 (1/2/3/4/5)
    label_gear_display_ = new QLabel("-", this);
    label_gear_display_->setFont(QFont("", 20, QFont::Bold));
    label_gear_display_->setAlignment(Qt::AlignCenter);
    label_gear_display_->setMinimumSize(50, 50);
    label_gear_display_->setStyleSheet("background:white;color:#007AFF;border-radius:10px;border:2px solid #007AFF;");
    btn_gear_up_ = createHoldButton("\u5347\u6321 \u25B2", STY_GRAY);
    syL->addWidget(btn_unlock_); syL->addWidget(btn_stop_); syL->addWidget(btn_brake_);
    syL->addWidget(label_gear_dir_display_); syL->addWidget(btn_gear_down_); syL->addWidget(label_gear_display_); syL->addWidget(btn_gear_up_);
    wL->addLayout(syL, 4,0,1,3);
    // 无人/有人模式切换
    auto* modeL2 = new QHBoxLayout();
    // "有人/无人模式" — iOS SmoothToggle + 左标签 + 右状态文字
    auto* unmanLbl = new QLabel("\u65E0\u4EBA\u6A21\u5F0F"); unmanLbl->setFont(QFont("", 12, QFont::Bold));
    btn_unmanned_mode_ = new SmoothToggle();
    auto* unmanStatus = new QLabel("\u6709\u4EBA");
    unmanStatus->setFont(MONO_FONT);
    unmanStatus->setStyleSheet("color:#8E8E93;");
    connect(btn_unmanned_mode_, &SmoothToggle::toggled, [unmanStatus](bool on){
        unmanStatus->setText(on ? "\u65E0\u4EBA" : "\u6709\u4EBA");
        unmanStatus->setStyleSheet(on ? "color:#34C759;font-weight:bold;" : "color:#8E8E93;");
    });
    modeL2->addWidget(unmanLbl);
    modeL2->addWidget(btn_unmanned_mode_);
    modeL2->addWidget(unmanStatus);
    modeL2->addStretch();
    wL->addLayout(modeL2, 5,0,1,3);
    layout->addWidget(wG);

    auto* bG = makeGroup("\u94F2\u5200\u63A7\u5236 (\u6309\u4F4F\u6709\u6548, \u677E\u5F00\u505C\u6B62)");
    auto* bL = new QGridLayout(bG);
    btn_blade_up_         = new QPushButton("\u25B2", this); btn_blade_up_->setStyleSheet(circStyle + "QPushButton{background:#FF3B30;color:white;}QPushButton:pressed{background:#C02825;}"); attachCircleShadow(btn_blade_up_);
    btn_blade_down_       = new QPushButton("\u25BC", this); btn_blade_down_->setStyleSheet(circStyle + "QPushButton{background:#007AFF;color:white;}QPushButton:pressed{background:#0055B3;}"); attachCircleShadow(btn_blade_down_);
    btn_blade_tilt_left_  = new QPushButton("\u25C0", this); btn_blade_tilt_left_->setStyleSheet(circStyle + "QPushButton{background:#007AFF;color:white;}QPushButton:pressed{background:#0055B3;}"); attachCircleShadow(btn_blade_tilt_left_);
    btn_blade_tilt_right_ = new QPushButton("\u25B6", this); btn_blade_tilt_right_->setStyleSheet(circStyle + "QPushButton{background:#007AFF;color:white;}QPushButton:pressed{background:#0055B3;}"); attachCircleShadow(btn_blade_tilt_right_);
    auto* bladeIcon = new QLabel("\u26CF", this);
    bladeIcon->setAlignment(Qt::AlignCenter);
    bladeIcon->setFont(QFont("", 24));
    bladeIcon->setStyleSheet("background:#F9FAFB;border-radius:30px;min-width:60px;min-height:60px;max-width:60px;max-height:60px;");
    bL->addWidget(btn_blade_up_, 0,1, Qt::AlignCenter);
    bL->addWidget(btn_blade_tilt_left_, 1,0, Qt::AlignCenter);
    bL->addWidget(bladeIcon, 1,1, Qt::AlignCenter);
    bL->addWidget(btn_blade_tilt_right_, 1,2, Qt::AlignCenter);
    bL->addWidget(btn_blade_down_, 2,1, Qt::AlignCenter);
    auto* bcL = new QHBoxLayout();
    bcL->addWidget(new QLabel("速度:"));
    slider_blade_speed_ = new SmoothSlider(Qt::Horizontal); slider_blade_speed_->setRange(100,1000); slider_blade_speed_->setValue(500);
    bcL->addWidget(slider_blade_speed_);
    label_blade_speed_val_ = new QLabel("500"); bcL->addWidget(label_blade_speed_val_);
    chk_blade_enable_ = new QCheckBox("铲刀使能"); bcL->addWidget(chk_blade_enable_);
    bL->addLayout(bcL, 3,0,1,3);
    layout->addWidget(bG);
    layout->addStretch();
    tabs->addTab(tab, "\U0001F3AE \u76F4\u63A5CAN");
}

//==============================================================================
// Tab2: 节点功能测试 (和之前一样)
//==============================================================================
void ManualControlWidget::setupTab2_NodeTest(QTabWidget* tabs) {
    auto* tab = new QWidget();
    auto* layout = new QVBoxLayout(tab);

    auto* mG = makeGroup("\u6D4B\u8BD5\u6A21\u5F0F");
    auto* mL = new QHBoxLayout(mG);
    // iOS SegmentedControl: 3 选 1 模式切换, 白色滑块弹簧滑动 (融合风第八部分).
    // 替换掉原来的"3 个 QPushButton + 隐藏 QRadio + QButtonGroup"手工方案.
    mode_group_ = new SegmentedControl({
        "\u81EA\u52A8\u4F5C\u4E1A",    // 自动作业
        "\u5355\u6B65\u884C\u8D70",    // 单步行走
        "\u94F2\u5200\u627E\u5E73"     // 铲刀找平
    });
    mode_group_->setMinimumHeight(40);
    mL->addWidget(mode_group_);
    layout->addWidget(mG);

    group_auto_ = makeGroup("自动作业 — 主开关 + 前置条件 → 状态机运行");
    auto* aL = new QHBoxLayout(group_auto_);
    // "主开关" — iOS 51×31 SmoothToggle + 左侧标签 (开启绿色 #34C759)
    auto* mainSwLbl = new QLabel("主开关"); mainSwLbl->setFont(QFont("", 12, QFont::Bold));
    btn_node_main_switch_ = new SmoothToggle();
    auto* mainSwStatus = new QLabel("OFF"); mainSwStatus->setFont(MONO_FONT);
    mainSwStatus->setStyleSheet("color:#8E8E93;");
    // "感知就绪" — iOS 51×31 SmoothToggle
    auto* detSwLbl = new QLabel("感知就绪"); detSwLbl->setFont(QFont("", 12, QFont::Bold));
    btn_node_detection_ = new SmoothToggle();
    auto* detSwStatus = new QLabel("未就绪"); detSwStatus->setFont(MONO_FONT);
    detSwStatus->setStyleSheet("color:#8E8E93;");
    // 动态更新状态文字, 绿色=开启, 灰色=关闭
    connect(btn_node_main_switch_, &SmoothToggle::toggled, [mainSwStatus](bool on){
        mainSwStatus->setText(on ? "ON" : "OFF");
        mainSwStatus->setStyleSheet(on ? "color:#34C759;font-weight:bold;" : "color:#8E8E93;");
    });
    connect(btn_node_detection_, &SmoothToggle::toggled, [detSwStatus](bool on){
        detSwStatus->setText(on ? "就绪" : "未就绪");
        detSwStatus->setStyleSheet(on ? "color:#34C759;font-weight:bold;" : "color:#8E8E93;");
    });
    aL->addWidget(mainSwLbl); aL->addWidget(btn_node_main_switch_); aL->addWidget(mainSwStatus);
    aL->addSpacing(24);
    aL->addWidget(detSwLbl);  aL->addWidget(btn_node_detection_);  aL->addWidget(detSwStatus);
    aL->addStretch();
    layout->addWidget(group_auto_);

    group_walk_ = makeGroup("单步行走 — 手动驱动 control_node");
    auto* wkL = new QGridLayout(group_walk_);
    btn_node_drive_ = new IOSButton("直行", IOSButton::Primary); btn_node_drive_->setMinimumHeight(40);
    btn_node_rotate_ = new IOSButton("旋转", IOSButton::Primary); btn_node_rotate_->setMinimumHeight(40);
    btn_node_stop_ = new IOSButton("停止", IOSButton::Danger); btn_node_stop_->setMinimumHeight(40);
    wkL->addWidget(btn_node_drive_,0,0); wkL->addWidget(btn_node_rotate_,0,1); wkL->addWidget(btn_node_stop_,0,2);
    wkL->addWidget(new QLabel("X终点(m):"),1,0);
    spin_x_terminal_ = makeSpin(-100,100,5.0,0.5,1); wkL->addWidget(spin_x_terminal_,1,1);
    wkL->addWidget(new QLabel("角度(°):"),2,0);
    spin_theta_terminal_ = makeSpin(-180,180,30,5,1); wkL->addWidget(spin_theta_terminal_,2,1);
    wkL->addWidget(new QLabel("线速(m/s):"),1,2);
    spin_v_ref_ = makeSpin(0,3,0.5,0.1,2); wkL->addWidget(spin_v_ref_,1,3);
    wkL->addWidget(new QLabel("角速(°/s):"),2,2);
    spin_omega_ref_ = makeSpin(0,30,5,1,1); wkL->addWidget(spin_omega_ref_,2,3);
    group_walk_->setVisible(false);
    layout->addWidget(group_walk_);

    group_blade_ = makeGroup("铲刀找平 — control_node PID → /U_Lever_Moldboard → CAN");
    auto* mdL = new QGridLayout(group_blade_);
    chk_node_mold_enable_ = new QCheckBox("铲刀控制使能"); mdL->addWidget(chk_node_mold_enable_,0,0,1,2);
    mdL->addWidget(new QLabel("目标高度(m):"),1,0);
    spin_ref_height_ = makeSpin(-5,5,0,0.01,3); mdL->addWidget(spin_ref_height_,1,1);
    mdL->addWidget(new QLabel("目标角度(°):"),1,2);
    spin_ref_angle_ = makeSpin(-30,30,0,0.5,1); mdL->addWidget(spin_ref_angle_,1,3);
    group_blade_->setVisible(false);
    layout->addWidget(group_blade_);

    // [Issue#7] 铲刀高度基准点设置
    group_blade_origin_ = makeGroup("铲刀高度基准点 (INIT_ORIGIN)");
    auto* boL = new QGridLayout(group_blade_origin_);
    boL->setVerticalSpacing(5);
    boL->addWidget(new QLabel("纬度:"), 0, 0);
    spin_blade_origin_lat_ = makeSpin(-90, 90, simulink::INIT_ORIGIN[0], 0.000000001, 9);
    spin_blade_origin_lat_->setDecimals(9);
    boL->addWidget(spin_blade_origin_lat_, 0, 1);
    boL->addWidget(new QLabel("经度:"), 0, 2);
    spin_blade_origin_lon_ = makeSpin(-180, 360, simulink::INIT_ORIGIN[1], 0.000000001, 9);
    spin_blade_origin_lon_->setDecimals(9);
    boL->addWidget(spin_blade_origin_lon_, 0, 3);
    boL->addWidget(new QLabel("海拔(m):"), 1, 0);
    spin_blade_origin_alt_ = makeSpin(-500, 9000, simulink::INIT_ORIGIN[2], 0.001, 3);
    boL->addWidget(spin_blade_origin_alt_, 1, 1);
    btn_lock_blade_origin_ = new IOSButton("锁定当前RTK位置", IOSButton::Primary);
    boL->addWidget(btn_lock_blade_origin_, 1, 2);
    auto* btn_send_blade_origin = new IOSButton("发送", IOSButton::Success);
    boL->addWidget(btn_send_blade_origin, 1, 3);
    label_blade_origin_status_ = new QLabel("默认: 标定场地");
    label_blade_origin_status_->setStyleSheet("color: #AEAEB2;");
    boL->addWidget(label_blade_origin_status_, 2, 0, 1, 4);
    group_blade_origin_->setVisible(false);
    layout->addWidget(group_blade_origin_);

    // 锁定当前RTK位置按钮: 取当前LLA填入编辑框
    connect(btn_lock_blade_origin_, &QPushButton::clicked, [this]() {
        if (lla_lat_ == 0 && lla_lon_ == 0) {
            label_blade_origin_status_->setText("RTK无信号, 无法锁定");
            label_blade_origin_status_->setStyleSheet("color: red;");
            return;
        }
        spin_blade_origin_lat_->setValue(lla_lat_);
        spin_blade_origin_lon_->setValue(lla_lon_);
        spin_blade_origin_alt_->setValue(lla_alt_);
        label_blade_origin_status_->setText(QString("已锁定: %1, %2, %3m")
            .arg(lla_lat_, 0, 'f', 9).arg(lla_lon_, 0, 'f', 9).arg(lla_alt_, 0, 'f', 3));
        label_blade_origin_status_->setStyleSheet("color: green;");
        // 自动发送
        geometry_msgs::Point msg;
        msg.x = lla_lat_; msg.y = lla_lon_; msg.z = lla_alt_;
        pub_set_blade_origin_.publish(msg);
    });

    // 手动发送按钮
    connect(btn_send_blade_origin, &QPushButton::clicked, [this]() {
        geometry_msgs::Point msg;
        msg.x = spin_blade_origin_lat_->value();
        msg.y = spin_blade_origin_lon_->value();
        msg.z = spin_blade_origin_alt_->value();
        pub_set_blade_origin_.publish(msg);
        label_blade_origin_status_->setText(QString("已发送: %1, %2, %3m")
            .arg(msg.x, 0, 'f', 9).arg(msg.y, 0, 'f', 9).arg(msg.z, 0, 'f', 3));
        label_blade_origin_status_->setStyleSheet("color: green;");
    });

    // 地图已移到顶层常驻区, Tab2不再包含地图

    auto* monG = makeGroup("节点输出");
    auto* moL = new QGridLayout(monG);
    label_v_right_ = new QLabel("V_right: --"); label_v_right_->setFont(MONO_FONT);
    label_v_left_ = new QLabel("V_left: --"); label_v_left_->setFont(MONO_FONT);
    label_terminal_flag_ = new QLabel("Terminal: --"); label_terminal_flag_->setFont(MONO_FONT);
    label_mold_debug_ = new QLabel("铲刀PID: --"); label_mold_debug_->setFont(MONO_FONT);
    moL->addWidget(label_v_right_,0,0); moL->addWidget(label_v_left_,0,1); moL->addWidget(label_terminal_flag_,0,2);
    moL->addWidget(label_mold_debug_,1,0,1,3);
    layout->addWidget(monG);

    log_output_ = new QTextEdit(tab);
    log_output_->setReadOnly(true); log_output_->setMaximumHeight(80); log_output_->setFont(MONO_FONT);
    layout->addWidget(log_output_);
    tabs->addTab(tab, "\U0001F9EA \u8282\u70B9\u6D4B\u8BD5");
}

//==============================================================================
// Tab3: PID调参
//==============================================================================
void ManualControlWidget::setupTab3_PIDTuning(QTabWidget* tabs)
{
    // 阶段三 · 布局重构: GroupedSection + ParameterRow + RubberScrollArea
    // 两组 PID 参数原本是 QGridLayout 平铺, 改成 iOS 分组列表风格.
    // 每行的"标签"用 ParamLabel (悬停 tooltip + 点击弹详细说明).
    auto* tab    = new QWidget();
    auto* scroll = new RubberScrollArea(tab);
    auto* inner  = new QWidget();
    inner->setStyleSheet(QStringLiteral("QWidget{background:%1;}")
                         .arg(Theme::hex(Theme::bgWindow)));
    auto* layout = new QVBoxLayout(inner);
    layout->setContentsMargins(Spacing::panelPadding, Spacing::panelPadding,
                               Spacing::panelPadding, 8);
    layout->setSpacing(Spacing::sectionGap);

    // ------------------------------------------------------------------
    // 行走 PID — 距离控制 + 航向控制 + 容差 + 滤波 + 死区
    // ------------------------------------------------------------------
    auto* walkSec = new GroupedSection(QString::fromUtf8(
        "\xe8\xa1\x8c\xe8\xb5\xb0 PID  (\xe7\x82\xb9\xe5\x87\xbb\xe5\x8f\x82\xe6\x95\xb0\xe5\x90\x8d\xe6\x9f\xa5\xe7\x9c\x8b\xe8\xaf\xb4\xe6\x98\x8e)"),
        inner);

    // ===== 距离控制 (V_right + V_left) =====
    spin_kp_x_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_x_)) s->setSafeRange(0.3, 5.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel("Kp",
            QString::fromUtf8("\xe8\xb7\x9d\xe7\xa6\xbb\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a \xe2\x80\x94 \xe8\xaf\xaf\xe5\xb7\xae\xe8\xb6\x8a\xe5\xa4\xa7\xe8\xbe\x93\xe5\x87\xba\xe8\xb6\x8a\xe5\xbc\xba"),
            QString::fromUtf8("<b>Kp\xef\xbc\x88\xe8\xb7\x9d\xe7\xa6\xbb\xe6\x8e\xa7\xe5\x88\xb6\xef\xbc\x89</b><br><br>"
                              "\xe6\x8e\xa7\xe5\x88\xb6\xe5\x8f\x98\xe9\x87\x8f\xef\xbc\x9a\xe5\x89\x8d\xe8\xbf\x9b/\xe5\x90\x8e\xe9\x80\x80\xe9\x80\x9f\xe5\xba\xa6\xe4\xb9\x8b\xe5\x92\x8c (V_right + V_left)<br>"
                              "\xe8\xaf\xaf\xe5\xb7\xae\xe5\xae\x9a\xe4\xb9\x89\xef\xbc\x9a\xe7\x9b\xae\xe6\xa0\x87\xe5\x89\x8d\xe8\xbf\x9b\xe8\xb7\x9d\xe7\xa6\xbb \xe2\x88\x92 \xe5\xbd\x93\xe5\x89\x8d\xe5\xb7\xb2\xe8\xb5\xb0\xe8\xb7\x9d\xe7\xa6\xbb\xef\xbc\x88m\xef\xbc\x89<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "1.0</i>")),
        spin_kp_x_, walkSec));

    spin_ki_x_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_x_)) s->setSafeRange(0.01, 1.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel("Ki",
            QString::fromUtf8("\xe8\xb7\x9d\xe7\xa6\xbb\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a \xe2\x80\x94 \xe6\xb6\x88\xe9\x99\xa4\xe5\x9d\xa1\xe9\x81\x93/\xe6\x89\x93\xe6\xbb\x91\xe7\xad\x89\xe6\x8c\x81\xe7\xbb\xad\xe5\x81\x8f\xe5\xb7\xae"),
            QString::fromUtf8("<b>Ki\xef\xbc\x88\xe8\xb7\x9d\xe7\xa6\xbb\xe6\x8e\xa7\xe5\x88\xb6\xef\xbc\x89</b><br><br>"
                              "\xe7\xa7\xaf\xe5\x88\x86\xe9\xa1\xb9\xe7\xb4\xaf\xe5\x8a\xa0\xe5\x8e\x86\xe5\x8f\xb2\xe8\xaf\xaf\xe5\xb7\xae\xef\xbc\x8c\xe8\xa1\xa5\xe5\x81\xbf\xe5\x9d\xa1\xe9\x81\x93\xe9\x98\xbb\xe5\x8a\x9b\xe7\xad\x89\xe5\xaf\xbc\xe8\x87\xb4\xe7\x9a\x84<b>\xe7\xa8\xb3\xe6\x80\x81\xe8\xaf\xaf\xe5\xb7\xae</b>\xe3\x80\x82<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "0.1</i>")),
        spin_ki_x_, walkSec));

    spin_kd_x_ = makeSpin(0, 100, 0.01, 0.001, 4);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kd_x_)) s->setSafeRange(0.0, 0.1);
    walkSec->addRow(new ParameterRow(
        makeParamLabel("Kd",
            QString::fromUtf8("\xe8\xb7\x9d\xe7\xa6\xbb\xe5\xbe\xae\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a \xe2\x80\x94 \xe6\x8a\x91\xe5\x88\xb6\xe8\xb6\x85\xe8\xb0\x83"),
            QString::fromUtf8("<b>Kd\xef\xbc\x88\xe8\xb7\x9d\xe7\xa6\xbb\xe6\x8e\xa7\xe5\x88\xb6\xef\xbc\x89</b><br><br>"
                              "\xe5\xaf\xb9\xe8\xaf\xaf\xe5\xb7\xae\xe5\x8f\x98\xe5\x8c\x96\xe7\x8e\x87\xe8\xbf\x9b\xe8\xa1\x8c\xe6\x8a\x91\xe5\x88\xb6\xef\xbc\x8c\xe5\x87\x8f\xe5\xb0\x91\xe8\xb6\x85\xe8\xb0\x83\xe3\x80\x82<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "0.01</i>")),
        spin_kd_x_, walkSec));

    // ===== 航向控制 (V_right - V_left) =====
    spin_kp_theta_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_theta_)) s->setSafeRange(0.5, 8.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91 Kp"),
            QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a \xe2\x80\x94 \xe6\x8e\xa7\xe5\x88\xb6\xe5\xb7\xae\xe9\x80\x9f\xe8\xbd\xac\xe5\x90\x91\xe5\xbc\xba\xe5\xba\xa6"),
            QString::fromUtf8("<b>\xe8\x88\xaa\xe5\x90\x91 Kp</b><br><br>"
                              "\xe6\x8e\xa7\xe5\x88\xb6\xe5\x8f\x98\xe9\x87\x8f\xef\xbc\x9a\xe5\xb7\xa6\xe5\x8f\xb3\xe5\xb1\xa5\xe5\xb8\xa6\xe9\x80\x9f\xe5\xba\xa6\xe4\xb9\x8b\xe5\xb7\xae (V_right \xe2\x88\x92 V_left)<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "1.0</i>")),
        spin_kp_theta_, walkSec));

    spin_ki_theta_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_theta_)) s->setSafeRange(0.01, 1.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91 Ki"),
            QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>\xe8\x88\xaa\xe5\x90\x91 Ki</b><br><br>"
                              "\xe8\xa1\xa5\xe5\x81\xbf\xe4\xb8\xa4\xe4\xbe\xa7\xe5\xb1\xa5\xe5\xb8\xa6\xe7\xa3\xa8\xe6\x8d\x9f\xe4\xb8\x8d\xe5\x9d\x87\xe5\xaf\xbc\xe8\x87\xb4\xe7\x9a\x84\xe6\x8c\x81\xe7\xbb\xad\xe5\x81\x8f\xe8\x88\xaa\xe3\x80\x82<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "0.1</i>")),
        spin_ki_theta_, walkSec));

    spin_kd_theta_ = makeSpin(0, 100, 0.01, 0.001, 4);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kd_theta_)) s->setSafeRange(0.0, 0.1);
    walkSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91 Kd"),
            QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91\xe5\xbe\xae\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>\xe8\x88\xaa\xe5\x90\x91 Kd</b><br><br>"
                              "\xe5\xaf\xb9\xe8\x88\xaa\xe5\x90\x91\xe8\xaf\xaf\xe5\xb7\xae\xe5\x8f\x98\xe5\x8c\x96\xe7\x8e\x87\xe8\xbf\x9b\xe8\xa1\x8c\xe9\x98\xbb\xe5\xb0\xbc\xe3\x80\x82<br><br>"
                              "<i>\xe9\xbb\x98\xe8\xae\xa4\xe5\x80\xbc\xef\xbc\x9a" "0.01</i>")),
        spin_kd_theta_, walkSec));

    // ===== 容差 / 履带宽 =====
    spin_x_tol_ = makeSpin(0, 5, 0.1, 0.05, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_x_tol_)) s->setSafeRange(0.05, 0.5);
    {
        auto* row = new ParameterRow(
            makeParamLabel(QString::fromUtf8("X \xe5\xae\xb9\xe5\xb7\xae"),
                QString::fromUtf8("\xe5\x88\xb0\xe4\xbd\x8d\xe5\x88\xa4\xe5\xae\x9a\xe8\xb7\x9d\xe7\xa6\xbb\xe9\x98\x88\xe5\x80\xbc"),
                QString::fromUtf8("<b>X\xe5\xae\xb9\xe5\xb7\xae</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.1 m")),
            spin_x_tol_, walkSec);
        row->setUnit("m");
        walkSec->addRow(row);
    }

    spin_theta_tol_ = makeSpin(0, 30, 2.0, 0.5, 1);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_theta_tol_)) s->setSafeRange(1.0, 5.0);
    {
        auto* row = new ParameterRow(
            makeParamLabel(QString::fromUtf8("\xce\xb8 \xe5\xae\xb9\xe5\xb7\xae"),
                QString::fromUtf8("\xe5\x88\xb0\xe4\xbd\x8d\xe5\x88\xa4\xe5\xae\x9a\xe8\xa7\x92\xe5\xba\xa6\xe9\x98\x88\xe5\x80\xbc"),
                QString::fromUtf8("<b>\xce\xb8\xe5\xae\xb9\xe5\xb7\xae</b><br>\xe9\xbb\x98\xe8\xae\xa4 2.0\xc2\xb0")),
            spin_theta_tol_, walkSec);
        row->setUnit(QString::fromUtf8("\xc2\xb0"));
        walkSec->addRow(row);
    }

    spin_track_width_ = makeSpin(0.5, 5, 2.5, 0.1, 1);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_track_width_)) s->setSafeRange(2.0, 3.5);
    {
        auto* row = new ParameterRow(
            makeParamLabel(QString::fromUtf8("\xe5\xb1\xa5\xe5\xb8\xa6\xe5\xae\xbd"),
                QString::fromUtf8("\xe4\xb8\xa4\xe4\xbe\xa7\xe5\xb1\xa5\xe5\xb8\xa6\xe4\xb8\xad\xe5\xbf\x83\xe7\xba\xbf\xe9\x97\xb4\xe8\xb7\x9d (\xe5\xb7\xae\xe9\x80\x9f\xe5\x9f\xba\xe8\xb7\x9d)"),
                QString::fromUtf8("<b>\xe5\xb1\xa5\xe5\xb8\xa6\xe5\xae\xbd</b><br>"
                                  "V_right = V + \xcf\x89 \xc3\x97 width / 2<br>"
                                  "V_left&nbsp;= V \xe2\x88\x92 \xcf\x89 \xc3\x97 width / 2<br>"
                                  "\xe6\x8d\xa2\xe9\xa1\xb9\xe7\x9b\xae\xe5\xbf\x85\xe9\xa1\xbb\xe9\x87\x8d\xe6\x96\xb0\xe6\xb5\x8b\xe9\x87\x8f\xe3\x80\x82")),
            spin_track_width_, walkSec);
        row->setUnit("m");
        walkSec->addRow(row);
    }

    // ===== TD 滤波 =====
    spin_td_r_ = makeSpin(0, 1000, 100, 10, 0);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_td_r_)) s->setSafeRange(20.0, 300.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel("TD_r",
            QString::fromUtf8("\xe8\xb7\x9f\xe8\xb8\xaa\xe5\xbe\xae\xe5\x88\x86\xe5\x99\xa8\xe9\x80\x9f\xe5\xba\xa6\xe5\x9b\xa0\xe5\xad\x90"),
            QString::fromUtf8("<b>TD_r</b><br>\xe9\xbb\x98\xe8\xae\xa4 100, \xe6\x8e\xa8\xe5\x9c\x9f\xe6\x9c\xba\xe5\xbb\xba\xe8\xae\xae 100")),
        spin_td_r_, walkSec));

    spin_td_h_ = makeSpin(0, 1, 0.01, 0.001, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_td_h_)) s->setSafeRange(0.005, 0.05);
    walkSec->addRow(new ParameterRow(
        makeParamLabel("TD_h",
            QString::fromUtf8("\xe8\xb7\x9f\xe8\xb8\xaa\xe5\xbe\xae\xe5\x88\x86\xe5\x99\xa8\xe6\xbb\xa4\xe6\xb3\xa2\xe6\xad\xa5\xe9\x95\xbf"),
            QString::fromUtf8("<b>TD_h</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.01")),
        spin_td_h_, walkSec));

    // ===== 桥接死区 =====
    spin_gear_dz_ = makeSpin(0, 100, 10, 1, 0);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_gear_dz_)) s->setSafeRange(5.0, 30.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe6\x8c\xa1\xe4\xbd\x8d\xe6\xad\xbb\xe5\x8c\xba"),
            QString::fromUtf8("\xe4\xbd\x8e\xe4\xba\x8e\xe6\xad\xa4\xe5\x80\xbc\xe8\xbe\x93\xe5\x87\xba\xe7\xa9\xba\xe6\x8c\xa1, \xe9\x98\xb2\xe5\xa4\xb1\xe9\xa2\x91\xe5\x88\x87\xe6\x8c\xa1"),
            QString::fromUtf8("<b>\xe6\x8c\xa1\xe4\xbd\x8d\xe6\xad\xbb\xe5\x8c\xba</b><br>\xe9\xbb\x98\xe8\xae\xa4 10")),
        spin_gear_dz_, walkSec));

    spin_steer_dz_ = makeSpin(0, 200, 20, 5, 0);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_steer_dz_)) s->setSafeRange(10.0, 60.0);
    walkSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xbd\xac\xe5\x90\x91\xe6\xad\xbb\xe5\x8c\xba"),
            QString::fromUtf8("\xe4\xbd\x8e\xe4\xba\x8e\xe6\xad\xa4\xe5\x80\xbc\xe4\xb8\x8d\xe5\x8f\x91\xe8\xbd\xac\xe5\x90\x91, \xe6\xb6\x88\xe9\x99\xa4\xe6\xb6\xb2\xe5\x8e\x8b\xe6\x8a\x96\xe5\x8a\xa8"),
            QString::fromUtf8("<b>\xe8\xbd\xac\xe5\x90\x91\xe6\xad\xbb\xe5\x8c\xba</b><br>\xe9\xbb\x98\xe8\xae\xa4 20")),
        spin_steer_dz_, walkSec));
    layout->addWidget(walkSec);

    // 行走 PID 发送按钮 (保持原视觉位置: 紧跟着 walkSec)
    btn_send_walk_params_ = new IOSButton(QString::fromUtf8("\xe5\x8f\x91\xe9\x80\x81\xe8\xa1\x8c\xe8\xb5\xb0\xe5\x8f\x82\xe6\x95\xb0"),
                                           IOSButton::Success);
    btn_send_walk_params_->setMinimumHeight(40);
    label_walk_params_ack_ = new QLabel("");
    label_walk_params_ack_->setFont(MONO_FONT);
    {
        auto* btnRow = new QHBoxLayout();
        btnRow->setContentsMargins(0, 0, 0, 0);
        btnRow->addWidget(btn_send_walk_params_);
        btnRow->addWidget(label_walk_params_ack_, 1);
        auto* wrap = new QWidget(inner);
        wrap->setLayout(btnRow);
        layout->addWidget(wrap);
    }

    // ------------------------------------------------------------------
    // 行走 PID 自动整定 (保持原实现, 只是放进同一布局)
    // ------------------------------------------------------------------
    setupTab3_WalkAutoTune(layout);

    // ------------------------------------------------------------------
    // 铲刀 PID — 高度控制 + 角度控制
    // ------------------------------------------------------------------
    auto* moldSec = new GroupedSection(QString::fromUtf8(
        "\xe9\x93\xb2\xe5\x88\x80 PID  (\xe7\x82\xb9\xe5\x87\xbb\xe5\x8f\x82\xe6\x95\xb0\xe5\x90\x8d\xe6\x9f\xa5\xe7\x9c\x8b\xe8\xaf\xb4\xe6\x98\x8e)"),
        inner);

    // ===== 高度控制 =====
    spin_kp_h_up_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_h_up_)) s->setSafeRange(0.3, 5.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6 Kp\xe5\x8d\x87"),
            QString::fromUtf8("\xe9\x93\xb2\xe5\x88\x80\xe4\xb8\x8a\xe5\x8d\x87\xe6\x97\xb6\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Kp\xe5\x8d\x87(\xe9\xab\x98\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 1.0")),
        spin_kp_h_up_, moldSec));

    spin_kp_h_dn_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_h_dn_)) s->setSafeRange(0.3, 5.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6 Kp\xe9\x99\x8d"),
            QString::fromUtf8("\xe9\x93\xb2\xe5\x88\x80\xe4\xb8\x8b\xe9\x99\x8d\xe6\x97\xb6\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Kp\xe9\x99\x8d(\xe9\xab\x98\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 1.0")),
        spin_kp_h_dn_, moldSec));

    spin_dz_height_ = makeSpin(0, 50, 3.0, 0.5, 1);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_dz_height_)) s->setSafeRange(1.0, 8.0);
    {
        auto* row = new ParameterRow(
            makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6\xe6\xad\xbb\xe5\x8c\xba"),
                QString::fromUtf8("\xe8\xaf\xaf\xe5\xb7\xae\xe5\xb0\x8f\xe4\xba\x8e\xe6\xad\xa4\xe5\x80\xbc\xe5\x81\x9c\xe6\xad\xa2\xe5\x8a\xa8\xe4\xbd\x9c"),
                QString::fromUtf8("<b>\xe9\xab\x98\xe5\xba\xa6\xe6\xad\xbb\xe5\x8c\xba</b><br>\xe9\xbb\x98\xe8\xae\xa4 3 mm")),
            spin_dz_height_, moldSec);
        row->setUnit("mm");
        moldSec->addRow(row);
    }

    spin_ki_h_up_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_h_up_)) s->setSafeRange(0.01, 1.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6 Ki\xe5\x8d\x87"),
            QString::fromUtf8("\xe4\xb8\x8a\xe5\x8d\x87\xe6\x96\xb9\xe5\x90\x91\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Ki\xe5\x8d\x87(\xe9\xab\x98\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.1")),
        spin_ki_h_up_, moldSec));

    spin_ki_h_dn_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_h_dn_)) s->setSafeRange(0.01, 1.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6 Ki\xe9\x99\x8d"),
            QString::fromUtf8("\xe4\xb8\x8b\xe9\x99\x8d\xe6\x96\xb9\xe5\x90\x91\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Ki\xe9\x99\x8d(\xe9\xab\x98\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.1")),
        spin_ki_h_dn_, moldSec));

    spin_imax_height_ = makeSpin(0, 5000, 500, 50, 0);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_imax_height_)) s->setSafeRange(200.0, 1500.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe9\xab\x98\xe5\xba\xa6 I\xe9\x99\x90\xe5\xb9\x85"),
            QString::fromUtf8("\xe7\xa7\xaf\xe5\x88\x86\xe9\xa1\xb9\xe7\xbb\x9d\xe5\xaf\xb9\xe5\x80\xbc\xe4\xb8\x8a\xe9\x99\x90"),
            QString::fromUtf8("<b>I\xe9\x99\x90\xe5\xb9\x85(\xe9\xab\x98\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 500")),
        spin_imax_height_, moldSec));

    // ===== 角度控制 =====
    spin_kp_t_up_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_t_up_)) s->setSafeRange(0.3, 5.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6 Kp\xe5\x8d\x87"),
            QString::fromUtf8("\xe5\x80\xbe\xe6\x96\x9c\xe5\xa2\x9e\xe5\xa4\xa7\xe6\x97\xb6\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Kp\xe5\x8d\x87(\xe8\xa7\x92\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 1.0")),
        spin_kp_t_up_, moldSec));

    spin_kp_t_dn_ = makeSpin(0, 100, 1.0, 0.1, 2);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_kp_t_dn_)) s->setSafeRange(0.3, 5.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6 Kp\xe9\x99\x8d"),
            QString::fromUtf8("\xe5\x80\xbe\xe6\x96\x9c\xe5\xa4\x8d\xe4\xbd\x8d\xe6\x97\xb6\xe6\xaf\x94\xe4\xbe\x8b\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Kp\xe9\x99\x8d(\xe8\xa7\x92\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 1.0")),
        spin_kp_t_dn_, moldSec));

    spin_dz_theta_ = makeSpin(0, 50, 3.0, 0.5, 1);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_dz_theta_)) s->setSafeRange(1.0, 8.0);
    {
        auto* row = new ParameterRow(
            makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6\xe6\xad\xbb\xe5\x8c\xba"),
                QString::fromUtf8("\xe6\xb6\x88\xe9\x99\xa4 IMU \xe5\x99\xaa\xe5\xa3\xb0\xe5\xbc\x95\xe8\xb5\xb7\xe7\x9a\x84\xe6\x8a\x96\xe5\x8a\xa8"),
                QString::fromUtf8("<b>\xe8\xa7\x92\xe5\xba\xa6\xe6\xad\xbb\xe5\x8c\xba</b><br>\xe9\xbb\x98\xe8\xae\xa4 3.0\xc2\xb0")),
            spin_dz_theta_, moldSec);
        row->setUnit(QString::fromUtf8("\xc2\xb0"));
        moldSec->addRow(row);
    }

    spin_ki_t_up_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_t_up_)) s->setSafeRange(0.01, 1.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6 Ki\xe5\x8d\x87"),
            QString::fromUtf8("\xe5\x80\xbe\xe6\x96\x9c\xe5\xa2\x9e\xe5\xa4\xa7\xe6\x96\xb9\xe5\x90\x91\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Ki\xe5\x8d\x87(\xe8\xa7\x92\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.1")),
        spin_ki_t_up_, moldSec));

    spin_ki_t_dn_ = makeSpin(0, 100, 0.1, 0.01, 3);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_ki_t_dn_)) s->setSafeRange(0.01, 1.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6 Ki\xe9\x99\x8d"),
            QString::fromUtf8("\xe5\x80\xbe\xe6\x96\x9c\xe5\x87\x8f\xe5\xb0\x8f\xe6\x96\xb9\xe5\x90\x91\xe7\xa7\xaf\xe5\x88\x86\xe5\xa2\x9e\xe7\x9b\x8a"),
            QString::fromUtf8("<b>Ki\xe9\x99\x8d(\xe8\xa7\x92\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 0.1")),
        spin_ki_t_dn_, moldSec));

    spin_imax_theta_ = makeSpin(0, 5000, 500, 50, 0);
    if (auto* s = qobject_cast<SmartSpinBox*>(spin_imax_theta_)) s->setSafeRange(200.0, 1500.0);
    moldSec->addRow(new ParameterRow(
        makeParamLabel(QString::fromUtf8("\xe8\xa7\x92\xe5\xba\xa6 I\xe9\x99\x90\xe5\xb9\x85"),
            QString::fromUtf8("\xe7\xa7\xaf\xe5\x88\x86\xe9\xa1\xb9\xe7\xbb\x9d\xe5\xaf\xb9\xe5\x80\xbc\xe4\xb8\x8a\xe9\x99\x90"),
            QString::fromUtf8("<b>I\xe9\x99\x90\xe5\xb9\x85(\xe8\xa7\x92\xe5\xba\xa6)</b><br>\xe9\xbb\x98\xe8\xae\xa4 500")),
        spin_imax_theta_, moldSec));
    layout->addWidget(moldSec);

    // 铲刀 PID 发送按钮
    btn_send_mold_params_ = new IOSButton(QString::fromUtf8("\xe5\x8f\x91\xe9\x80\x81\xe9\x93\xb2\xe5\x88\x80\xe5\x8f\x82\xe6\x95\xb0"),
                                           IOSButton::Success);
    btn_send_mold_params_->setMinimumHeight(40);
    label_mold_params_ack_ = new QLabel("");
    label_mold_params_ack_->setFont(MONO_FONT);
    {
        auto* btnRow = new QHBoxLayout();
        btnRow->setContentsMargins(0, 0, 0, 0);
        btnRow->addWidget(btn_send_mold_params_);
        btnRow->addWidget(label_mold_params_ack_, 1);
        auto* wrap = new QWidget(inner);
        wrap->setLayout(btnRow);
        layout->addWidget(wrap);
    }

    // ------------------------------------------------------------------
    // 铲刀 PID 自动整定
    // ------------------------------------------------------------------
    setupTab3_AutoTune(layout);

    layout->addStretch();
    scroll->setWidget(inner);
    auto* tabL = new QVBoxLayout(tab);
    tabL->setContentsMargins(0,0,0,0);
    tabL->addWidget(scroll);
    tabs->addTab(tab, "\U0001F39B PID调参");
}

//==============================================================================
// Tab4: 整车状态
//==============================================================================
void ManualControlWidget::setupTab4_VehicleStatus(QTabWidget* tabs) {
    auto* tab = new QWidget();
    auto* scroll = new QScrollArea(tab);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* inner = new QWidget();
    inner->setStyleSheet("QWidget{background:#F9FAFB;}");

    auto* root = new QVBoxLayout(inner);
    root->setSpacing(10);
    root->setContentsMargins(14, 12, 14, 12);

    // --- ① 故障报警横幅 (置顶, 无故障时绿色, 有故障时红色) ---
    label_fault_summary_ = new QLabel("\u2705  \u65E0\u6545\u969C");
    label_fault_summary_->setAlignment(Qt::AlignCenter);
    label_fault_summary_->setMinimumHeight(44);
    label_fault_summary_->setStyleSheet(
        "QLabel{background:#E8F9EC;color:#2FA350;border:1px solid #A1E0AF;"
        "border-radius:10px;font-size:13px;font-weight:bold;padding:8px;}");
    root->addWidget(label_fault_summary_);

    // --- ② 核心动力仪表 (4 个大圆仪表) ---
    auto* gaugeRow = new QHBoxLayout();
    gaugeRow->setSpacing(10);

    gauge_engine_rpm_ = new CircularGauge("\u53D1\u52A8\u673A\u8F6C\u901F", "rpm", tab);
    gauge_engine_rpm_->setRange(0, 2500);
    gauge_engine_rpm_->setThresholds(2000, 2300);
    gauge_engine_rpm_->setDecimals(0);

    gauge_vehicle_speed_ = new CircularGauge("\u8F66\u901F", "m/s", tab);
    gauge_vehicle_speed_->setRange(0, 5.0);
    gauge_vehicle_speed_->setThresholds(3.5, 4.5);
    gauge_vehicle_speed_->setDecimals(2);

    gauge_coolant_temp_ = new CircularGauge("\u6C34\u6E29", "\u00B0C", tab);
    gauge_coolant_temp_->setRange(0, 120);
    gauge_coolant_temp_->setThresholds(95, 105);
    gauge_coolant_temp_->setDecimals(1);

    gauge_fuel_level_ = new CircularGauge("\u71C3\u6CB9", "%", tab);
    gauge_fuel_level_->setRange(0, 100);
    gauge_fuel_level_->setThresholds(20, 10, /*lowIsDanger=*/true);
    gauge_fuel_level_->setDecimals(0);

    gaugeRow->addWidget(gauge_engine_rpm_, 1);
    gaugeRow->addWidget(gauge_vehicle_speed_, 1);
    gaugeRow->addWidget(gauge_coolant_temp_, 1);
    gaugeRow->addWidget(gauge_fuel_level_, 1);
    root->addLayout(gaugeRow);

    // --- ③ 姿态 (Pitch/Roll 水平仪) + 右侧状态胶囊 ---
    auto* midRow = new QHBoxLayout();
    midRow->setSpacing(10);

    auto* attCol = new QVBoxLayout();
    attCol->setSpacing(8);
    bar_pitch_ = new LevelBar("\u4FEF\u4EF0 Pitch", 30, tab);
    bar_roll_  = new LevelBar("\u6A2A\u6EDA Roll",  30, tab);
    attCol->addWidget(bar_pitch_);
    attCol->addWidget(bar_roll_);
    midRow->addLayout(attCol, 1);

    auto* pillCol = new QGridLayout();
    pillCol->setSpacing(8);
    pill_hand_brake_     = new StatusPill("\u624B\u5239", tab);
    pill_hydraulic_lock_ = new StatusPill("\u6DB2\u538B\u9501", tab);
    pill_brake_valve_    = new StatusPill("\u5236\u52A8\u9600", tab);
    pill_manual_auto_    = new StatusPill("\u63A7\u5236\u6A21\u5F0F", tab);
    pillCol->addWidget(pill_hand_brake_,     0, 0);
    pillCol->addWidget(pill_hydraulic_lock_, 0, 1);
    pillCol->addWidget(pill_brake_valve_,    1, 0);
    pillCol->addWidget(pill_manual_auto_,    1, 1);
    midRow->addLayout(pillCol, 1);
    root->addLayout(midRow);

    // --- ④ 详细信息卡 (3 列表格, 传动 / 定位 / 辅助) ---
    auto makeInfoCard = [&](const QString& title) -> std::pair<QWidget*, QGridLayout*> {
        auto* card = new QWidget(tab);
        card->setStyleSheet("QWidget{background:white;border:1px solid #E0E0E5;border-radius:10px;}");
        auto* v = new QVBoxLayout(card);
        v->setContentsMargins(12, 10, 12, 10);
        v->setSpacing(4);
        auto* tl = new QLabel(title, card);
        tl->setStyleSheet("color:#AEAEB2;font-size:11px;font-weight:bold;border:none;");
        v->addWidget(tl);
        auto* body = new QGridLayout();
        body->setVerticalSpacing(3);
        body->setHorizontalSpacing(8);
        body->setContentsMargins(0, 4, 0, 0);
        v->addLayout(body);
        return {card, body};
    };
    auto addRow = [](QGridLayout* grid, int row, const QString& key, QLabel* valLbl) {
        auto* k = new QLabel(key);
        k->setStyleSheet("color:#8E8E93;font-size:11px;border:none;background:transparent;");
        valLbl->setStyleSheet("color:#000000;font-size:12px;font-weight:bold;border:none;background:transparent;font-family:Monospace;");
        valLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid->addWidget(k, row, 0);
        grid->addWidget(valLbl, row, 1);
    };

    auto* infoRow = new QHBoxLayout();
    infoRow->setSpacing(10);

    // 传动卡
    auto [tCard, tBody] = makeInfoCard("\u4F20\u52A8\u7CFB\u7EDF");
    label_trans_speed_   = new QLabel("--");
    label_gear_pos_      = new QLabel("--");
    label_gear_dir_      = new QLabel("--");
    label_engine_torque_ = new QLabel("--");
    label_engine_hours_  = new QLabel("--");
    addRow(tBody, 0, "\u53D8\u901F\u7BB1\u8F6C\u901F", label_trans_speed_);
    addRow(tBody, 1, "\u6321\u4F4D",                   label_gear_pos_);
    addRow(tBody, 2, "\u65B9\u5411",                   label_gear_dir_);
    addRow(tBody, 3, "\u53D1\u52A8\u673A\u626D\u77E9", label_engine_torque_);
    addRow(tBody, 4, "\u53D1\u52A8\u673A\u5DE5\u65F6", label_engine_hours_);
    infoRow->addWidget(tCard, 1);

    // 定位卡
    auto [pCard, pBody] = makeInfoCard("\u5B9A\u4F4D\u4E0E\u59FF\u6001");
    label_lla_detail_     = new QLabel("--");
    label_heading_detail_ = new QLabel("--");
    label_lla_detail_->setStyleSheet("color:#000000;font-size:11px;font-family:Monospace;border:none;background:transparent;");
    label_lla_detail_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label_lla_detail_->setWordWrap(true);
    addRow(pBody, 0, "LLA",                     label_lla_detail_);
    addRow(pBody, 1, "\u822A\u5411",             label_heading_detail_);
    infoRow->addWidget(pCard, 1);

    // 辅助卡
    auto [oCard, oBody] = makeInfoCard("\u8F85\u52A9\u4FE1\u606F");
    label_handle_turn_ = new QLabel("--");
    label_fuel_total_  = new QLabel("--");
    label_output_curr_ = new QLabel("--");
    label_output_curr_->setStyleSheet("color:#000000;font-size:10px;font-family:Monospace;border:none;background:transparent;");
    label_output_curr_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    addRow(oBody, 0, "\u8F6C\u5411\u503C",       label_handle_turn_);
    addRow(oBody, 1, "\u603B\u6CB9\u8017",       label_fuel_total_);
    addRow(oBody, 2, "\u7535\u6D41 (\u00D74)",   label_output_curr_);
    infoRow->addWidget(oCard, 1);

    root->addLayout(infoRow);
    root->addStretch();

    scroll->setWidget(inner);
    auto* tabL = new QVBoxLayout(tab);
    tabL->setContentsMargins(0, 0, 0, 0);
    tabL->addWidget(scroll);
    tabs->addTab(tab, "\U0001F527 \u6574\u8F66\u72B6\u6001");
}

//==============================================================================
// Tab5: 规划参数
//==============================================================================
void ManualControlWidget::setupTab5_PlanningParams(QTabWidget* tabs) {
    // 阶段三 · 布局系统重构 (融合风补充篇·补充一/二/十)
    //   QScrollArea           → RubberScrollArea (顶/底阻尼 + iOS 细滚动指示器)
    //   QGroupBox+QGridLayout → GroupedSection + ParameterRow (iOS 分组列表)
    auto* tab    = new QWidget();
    auto* scroll = new RubberScrollArea(tab);
    auto* inner  = new QWidget();
    inner->setStyleSheet(QStringLiteral("QWidget{background:%1;}")
                         .arg(Theme::hex(Theme::bgWindow)));

    auto* layout = new QVBoxLayout(inner);
    layout->setContentsMargins(Spacing::panelPadding, Spacing::panelPadding,
                               Spacing::panelPadding, 8);
    layout->setSpacing(Spacing::sectionGap);

    // 紧急停止 (路径模式下方的执行栏会引用 — 提前创建)
    btn_path_stop_ = new IOSButton(QString::fromUtf8("\xe2\x9a\xa0 \xe7\xb4\xa7\xe6\x80\xa5\xe5\x81\x9c\xe6\xad\xa2"),
                                    IOSButton::Danger);
    btn_path_stop_->setMinimumHeight(40);

    //=== 执行状态 ============================================================
    {
        auto* sec = new GroupedSection(QString::fromUtf8("\xe6\x89\xa7\xe8\xa1\x8c\xe7\x8a\xb6\xe6\x80\x81"), inner);
        label_exec_state_ = new QLabel(QString::fromUtf8("\xe7\xa9\xba\xe9\x97\xb2"));
        label_exec_state_->setFont(MONO_FONT);
        label_exec_state_->setStyleSheet(
            "color:#AEAEB2;font-size:14px;font-weight:bold;padding:4px 8px;"
            "background:rgba(142,142,147,0.1);border-radius:6px;");
        sec->addRow(new ParameterRow(QString::fromUtf8("\xe7\x8a\xb6\xe6\x80\x81"),
                                     label_exec_state_, sec));

        label_wp_progress_ = new QLabel("WP: --/--");
        label_wp_progress_->setFont(MONO_FONT);
        sec->addRow(new ParameterRow(QString::fromUtf8("\xe8\xb7\xaf\xe5\xbe\x84\xe8\xbf\x9b\xe5\xba\xa6"),
                                     label_wp_progress_, sec));
        layout->addWidget(sec);
    }

    //=== 路径模式 ============================================================
    auto* modeSec = new GroupedSection(QString::fromUtf8("\xe2\x9a\x99 \xe8\xb7\xaf\xe5\xbe\x84\xe6\xa8\xa1\xe5\xbc\x8f"),
                                       inner);
    {
        combo_path_mode_ = new SegmentedControl({
            QString::fromUtf8("\xe5\xba\x95\xe7\xab\xaf\xe5\x89\x8d\xe8\xbf\x9b"),
            QString::fromUtf8("\xe9\xa1\xb6\xe7\xab\xaf"),
            QString::fromUtf8("\xe5\xba\x95\xe7\xab\xaf\xe5\x80\x92\xe8\xbd\xa6")});
        combo_path_mode_->setToolTip(QString::fromUtf8(
            "\xe5\xba\x95\xe7\xab\xaf\xe5\x89\x8d\xe8\xbf\x9b: \xe5\x88\xb0\xe5\xba\x95\xe7\xab\xaf\xe5\x80\x92\xe8\xbd\xa6\xe5\x9b\x9e\xe6\x9d\xa5\xe5\x86\x8d\xe6\x96\x9c\xe8\xbf\x87\xe5\x8e\xbb\n"
            "\xe9\xa1\xb6\xe7\xab\xaf:     \xe6\x8e\xa8\xe5\xae\x8c\xe7\x9b\xb4\xe6\x8e\xa5\xe6\x96\x9c\xe8\xbf\x87\xe5\x8e\xbb\n"
            "\xe5\xba\x95\xe7\xab\xaf\xe5\x80\x92\xe8\xbd\xa6: \xe5\x80\x92\xe8\xbd\xa6\xe6\x97\xb6\xe6\x96\x9c\xe7\x9d\x80\xe5\x80\x92\xe8\xbf\x87\xe5\x8e\xbb"));
        combo_path_mode_->setMinimumWidth(280);
        modeSec->addRow(new ParameterRow(
            QString::fromUtf8("\xe6\x8d\xa2\xe5\x88\x97\xe6\xa8\xa1\xe5\xbc\x8f"),
            combo_path_mode_, modeSec));

        chk_test_mode_ = new SmoothToggle();
        auto* tRow = new ParameterRow(
            QString::fromUtf8("\xe6\x97\xa0\xe5\x9c\xb0\xe5\x9b\xbe\xe6\xa8\xa1\xe5\xbc\x8f"),
            chk_test_mode_, modeSec);
        tRow->setHint(QString::fromUtf8(
            "\xe5\x90\xaf\xe7\x94\xa8\xe5\x90\x8e\xe4\xbd\xbf\xe7\x94\xa8\xe8\x84\x9a\xe6\x9c\xac/\xe6\x89\x8b\xe5\x8a\xa8\xe5\x8f\x82\xe6\x95\xb0"));
        modeSec->addRow(tRow);
    }

    // 无地图面板
    test_panel_ = new QWidget(modeSec);
    {
        auto* tpL = new QGridLayout(test_panel_);
        tpL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        tpL->setHorizontalSpacing(8); tpL->setVerticalSpacing(6);
        tpL->addWidget(new QLabel(QString::fromUtf8("\xe6\x8e\xa8\xe5\x9c\x9f\xe9\x95\xbf\xe5\xba\xa6(m):")), 0, 0);
        spin_push_length_ = makeSpin(1, 200, 20, 1, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_push_length_)) s->setSafeRange(5.0, 80.0);
        tpL->addWidget(spin_push_length_, 0, 1);
        tpL->addWidget(new QLabel(QString::fromUtf8("\xe5\x88\x97\xe6\x95\xb0:")), 0, 2);
        spin_test_columns_ = makeSpin(1, 100, 5, 1, 0);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_test_columns_)) s->setSafeRange(1.0, 20.0);
        tpL->addWidget(spin_test_columns_, 0, 3);
        tpL->addWidget(new QLabel(QString::fromUtf8("\xe6\xaf\x8f\xe5\x88\x97\xe9\x81\x8d\xe6\x95\xb0:")), 0, 4);
        spin_test_passes_ = makeSpin(1, 50, 1, 1, 0);
        tpL->addWidget(spin_test_passes_, 0, 5);
    }
    test_panel_->setVisible(false);
    modeSec->addRow(test_panel_);

    // 有地图面板
    map_panel_ = new QWidget(modeSec);
    {
        auto* mpL = new QVBoxLayout(map_panel_);
        mpL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        mpL->setSpacing(6);

        auto* mpRow1 = new QHBoxLayout();
        mpRow1->addWidget(new QLabel(QString::fromUtf8("\xe6\x8e\xa8\xe5\x9c\x9f\xe6\x96\xb9\xe5\x90\x91:"), map_panel_));
        spin_map_push_heading_ = new SegmentedControl(
            {QString::fromUtf8("\xe2\x86\x91\xe5\x8c\x97"),
             QString::fromUtf8("\xe2\x86\x92\xe4\xb8\x9c"),
             QString::fromUtf8("\xe2\x86\x93\xe5\x8d\x97"),
             QString::fromUtf8("\xe2\x86\x90\xe8\xa5\xbf")},
            map_panel_);
        spin_map_push_heading_->setToolTip(QString::fromUtf8(
            "\xe5\x90\x91\xe5\x8c\x97 \xe2\x86\x91 (row\xe5\xa2\x9e\xe5\xa4\xa7)\n"
            "\xe5\x90\x91\xe4\xb8\x9c \xe2\x86\x92 (col\xe5\xa2\x9e\xe5\xa4\xa7)\n"
            "\xe5\x90\x91\xe5\x8d\x97 \xe2\x86\x93 (row\xe5\x87\x8f\xe5\xb0\x8f)\n"
            "\xe5\x90\x91\xe8\xa5\xbf \xe2\x86\x90 (col\xe5\x87\x8f\xe5\xb0\x8f)"));
        spin_map_push_heading_->setFixedWidth(200);
        mpRow1->addWidget(spin_map_push_heading_);
        mpRow1->addSpacing(16);
        mpRow1->addWidget(new QLabel(QString::fromUtf8("\xe8\xb5\xb7\xe7\x82\xb9:"), map_panel_));
        combo_map_start_corner_ = new SegmentedControl(
            {QString::fromUtf8("\xe5\xb7\xa6\xe4\xb8\x8b"),
             QString::fromUtf8("\xe5\x8f\xb3\xe4\xb8\x8b"),
             QString::fromUtf8("\xe5\xb7\xa6\xe4\xb8\x8a"),
             QString::fromUtf8("\xe5\x8f\xb3\xe4\xb8\x8a")},
            map_panel_);
        combo_map_start_corner_->setFixedWidth(200);
        mpRow1->addWidget(combo_map_start_corner_);
        mpRow1->addStretch();
        label_map_info_ = new QLabel(QString::fromUtf8("\xe5\x9c\xb0\xe5\x9b\xbe: \xe6\x9c\xaa\xe6\x94\xb6\xe5\x88\xb0"));
        label_map_info_->setFont(MONO_FONT);
        label_map_info_->setStyleSheet("color:#8E8E93;");
        mpRow1->addWidget(label_map_info_);
        mpL->addLayout(mpRow1);

        auto* mpRow2 = new QHBoxLayout();
        mpRow2->addWidget(new QLabel(QString::fromUtf8("\xe5\x9c\xb0\xe5\x9b\xbe\xe7\xbc\xa9\xe6\x94\xbe\xe7\xb3\xbb\xe6\x95\xb0:")));
        spin_map_scale_ = makeSpin(0.1, 3.0, 1.0, 0.1, 1);
        mpRow2->addWidget(spin_map_scale_);
        auto* scaleHint = new QLabel(QString::fromUtf8(
            "<1\xe7\xbc\xa9\xe5\xb0\x8f\xe8\x8c\x83\xe5\x9b\xb4  1=\xe5\x8e\x9f\xe5\xa7\x8b  >1\xe6\x94\xbe\xe5\xa4\xa7\xe8\x8c\x83\xe5\x9b\xb4"));
        scaleHint->setStyleSheet("color:#AEAEB2;font-size:10px;");
        mpRow2->addWidget(scaleHint);
        mpRow2->addStretch();
        mpL->addLayout(mpRow2);
    }
    map_panel_->setVisible(true);
    modeSec->addRow(map_panel_);

    // 简单脚本编辑器
    script_panel_ = new QWidget(modeSec);
    {
        auto* spL = new QVBoxLayout(script_panel_);
        spL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        spL->setSpacing(6);
        auto* scriptHdr = new QLabel(QString::fromUtf8(
            "\xe7\xae\x80\xe5\x8d\x95\xe8\x84\x9a\xe6\x9c\xac (\xe6\xaf\x8f\xe8\xa1\x8c\xe4\xb8\x80\xe4\xb8\xaa\xe6\x8c\x87\xe4\xbb\xa4):"));
        scriptHdr->setStyleSheet("color:#007AFF;font-weight:bold;");
        spL->addWidget(scriptHdr);
        edit_test_script_ = new QTextEdit();
        edit_test_script_->setFont(MONO_FONT);
        edit_test_script_->setMaximumHeight(100);
        edit_test_script_->setPlaceholderText(QString::fromUtf8(
            "# \xe5\x8f\xaf\xe7\x94\xa8\xe6\x8c\x87\xe4\xbb\xa4:\n"
            "FORWARD 10   # \xe5\x89\x8d\xe8\xbf\x9b" "10m\n"
            "BACKWARD 5   # \xe5\x90\x8e\xe9\x80\x80" "5m\n"
            "ROTATE 90    # \xe5\xb7\xa6\xe8\xbd\xac" "90\xc2\xb0\n"
            "LOOP 5       # \xe5\xbe\xaa\xe7\x8e\xaf" "5\xe6\xac\xa1"));
        edit_test_script_->setPlainText("FORWARD 10\nBACKWARD 10\nLOOP -1");
        spL->addWidget(edit_test_script_);
        auto* scriptBtnRow = new QHBoxLayout();
        btn_run_script_ = new IOSButton(QString::fromUtf8("\xe2\x96\xb6 \xe6\x89\xa7\xe8\xa1\x8c\xe8\x84\x9a\xe6\x9c\xac"), IOSButton::Success);
        btn_run_script_->setMinimumHeight(35);
        btn_stop_script_ = new IOSButton(QString::fromUtf8("\xe2\x96\xa0 \xe5\x81\x9c\xe6\xad\xa2"), IOSButton::Danger);
        btn_stop_script_->setMinimumHeight(35);
        btn_stop_script_->setEnabled(false);
        label_script_status_ = new QLabel(QString::fromUtf8("\xe5\xbe\x85\xe5\x91\xbd"));
        label_script_status_->setFont(MONO_FONT);
        scriptBtnRow->addWidget(btn_run_script_);
        scriptBtnRow->addWidget(btn_stop_script_);
        scriptBtnRow->addWidget(label_script_status_);
        spL->addLayout(scriptBtnRow);
    }
    script_panel_->setVisible(false);
    modeSec->addRow(script_panel_);

    connect(chk_test_mode_, &SmoothToggle::toggled, this, [this](bool on) {
        test_panel_->setVisible(on);
        script_panel_->setVisible(on);
        map_panel_->setVisible(!on);
    });
    layout->addWidget(modeSec);

    //=== 路径参数 ============================================================
    auto* pSec = new GroupedSection(QString::fromUtf8("\xe8\xb7\xaf\xe5\xbe\x84\xe5\x8f\x82\xe6\x95\xb0"), inner);
    {
        spin_shift_angle_ = makeSpin(5, 80, 12, 1, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_shift_angle_)) s->setSafeRange(10.0, 50.0);
        auto* shiftRow = new ParameterRow(QString::fromUtf8("\xe6\x96\x9c\xe7\xa7\xbb\xe8\xa7\x92\xe5\xba\xa6"),
                                          spin_shift_angle_, pSec);
        shiftRow->setUnit(QString::fromUtf8("\xc2\xb0"));
        pSec->addRow(shiftRow);

        spin_push_heading_ = makeSpin(-180, 180, 0, 5, 1);
        auto* hRow = new ParameterRow(QString::fromUtf8("\xe6\x8e\xa8\xe5\x9c\x9f\xe6\x96\xb9\xe5\x90\x91"),
                                      spin_push_heading_, pSec);
        hRow->setUnit(QString::fromUtf8("\xc2\xb0"));
        pSec->addRow(hRow);

        // 列宽 / 铲刀宽度
        label_blade_width_tag_ = new QLabel(QString::fromUtf8("\xe5\x88\x97\xe5\xae\xbd"));   // 兼容旧引用
        spin_blade_width_ = makeSpin(0.5, 20, 4.2, 0.1, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_blade_width_)) s->setSafeRange(2.0, 6.0);
        auto* bwRow = new ParameterRow(QString::fromUtf8("\xe5\x88\x97\xe5\xae\xbd / \xe9\x93\xb2\xe5\x88\x80\xe5\xae\xbd\xe5\xba\xa6"),
                                       spin_blade_width_, pSec);
        bwRow->setUnit("m");
        pSec->addRow(bwRow);

        // 自动计算结果 (蓝色提示条)
        label_calc_width_ = new QLabel("");
        label_calc_width_->setFont(MONO_FONT);
        label_calc_width_->setStyleSheet(
            "color:#007AFF;padding:6px 12px;background:rgba(0,122,255,0.06);"
            "border-radius:8px;");
        auto* calcWrap = new QWidget(pSec);
        auto* calcLay  = new QHBoxLayout(calcWrap);
        calcLay->setContentsMargins(Spacing::panelPadding, 2, Spacing::panelPadding, 6);
        calcLay->addWidget(label_calc_width_);
        calcLay->addStretch();
        pSec->addRow(calcWrap);

        spin_v_push_ = makeSpin(0.05, 3, 0.5, 0.05, 2);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_v_push_)) s->setSafeRange(0.2, 1.5);
        auto* vpRow = new ParameterRow(QString::fromUtf8("\xe6\x8e\xa8\xe5\x9c\x9f\xe9\x80\x9f\xe5\xba\xa6"),
                                       spin_v_push_, pSec);
        vpRow->setUnit("m/s");
        pSec->addRow(vpRow);

        spin_v_reverse_ = makeSpin(0.05, 3, 0.8, 0.05, 2);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_v_reverse_)) s->setSafeRange(0.3, 1.8);
        auto* vrRow = new ParameterRow(QString::fromUtf8("\xe5\x80\x92\xe8\xbd\xa6\xe9\x80\x9f\xe5\xba\xa6"),
                                       spin_v_reverse_, pSec);
        vrRow->setUnit("m/s");
        pSec->addRow(vrRow);

        spin_omega_rotate_ = makeSpin(1, 60, 10, 1, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_omega_rotate_)) s->setSafeRange(3.0, 25.0);
        auto* omRow = new ParameterRow(QString::fromUtf8("\xe6\x97\x8b\xe8\xbd\xac\xe8\xa7\x92\xe9\x80\x9f"),
                                       spin_omega_rotate_, pSec);
        omRow->setUnit(QString::fromUtf8("\xc2\xb0/s"));
        pSec->addRow(omRow);

        spin_x_back_set_ = makeSpin(0, 20, 3.0, 0.5, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_x_back_set_)) s->setSafeRange(1.0, 10.0);
        auto* xbRow = new ParameterRow(QString::fromUtf8("\xe8\xbf\x87\xe8\xbd\xbd\xe5\x90\x8e\xe9\x80\x80"),
                                       spin_x_back_set_, pSec);
        xbRow->setUnit("m");
        pSec->addRow(xbRow);

        spin_path_pos_tol_ = makeSpin(0.05, 2, 0.3, 0.05, 2);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_path_pos_tol_)) s->setSafeRange(0.1, 0.8);
        auto* posRow = new ParameterRow(QString::fromUtf8("\xe5\x88\xb0\xe4\xbd\x8d\xe5\xae\xb9\xe5\xb7\xae"),
                                        spin_path_pos_tol_, pSec);
        posRow->setUnit("m");
        pSec->addRow(posRow);

        spin_path_theta_tol_ = makeSpin(1, 30, 5, 1, 1);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_path_theta_tol_)) s->setSafeRange(2.0, 10.0);
        auto* thRow = new ParameterRow(QString::fromUtf8("\xe8\x88\xaa\xe5\x90\x91\xe5\xae\xb9\xe5\xb7\xae"),
                                       spin_path_theta_tol_, pSec);
        thRow->setUnit(QString::fromUtf8("\xc2\xb0"));
        pSec->addRow(thRow);
    }
    layout->addWidget(pSec);

    // 模式切换的计算/显示更新逻辑 (与原实现等价)
    auto updateParamView = [this]() {
        bool is_bot = (combo_path_mode_->selectedIndex() != 1);
        spin_blade_width_->setVisible(true);
        if (!is_bot) {
            spin_shift_angle_->setEnabled(false);
            spin_shift_angle_->setStyleSheet("background:#F9FAFB;color:#AEAEB2;");
            double L = spin_push_length_->value();
            double w = spin_blade_width_->value();
            if (L > 0.01) {
                double angle_auto = std::atan(w / L) * 180.0 / M_PI;
                spin_shift_angle_->blockSignals(true);
                spin_shift_angle_->setValue(angle_auto);
                spin_shift_angle_->blockSignals(false);
                double d = std::sqrt(L * L + w * w);
                label_calc_width_->setText(QString::fromUtf8(
                    "\xe2\x86\x92 \xe8\xa7\x92\xe5\xba\xa6=%1\xc2\xb0(\xe8\x87\xaa\xe5\x8a\xa8)  \xe6\x96\x9c\xe7\xa7\xbb=%2m")
                    .arg(angle_auto, 0, 'f', 1).arg(d, 0, 'f', 2));
            }
        } else {
            spin_shift_angle_->setEnabled(true);
            spin_shift_angle_->setStyleSheet("");
            double a = spin_shift_angle_->value() * M_PI / 180.0;
            double w = spin_blade_width_->value();
            if (a > 0.01) {
                double d = w / std::sin(a);
                double fwd = w / std::tan(a);
                label_calc_width_->setText(QString::fromUtf8(
                    "\xe2\x86\x92 \xe6\x96\x9c\xe7\xa7\xbb=%1m  \xe7\xba\xb5\xe5\x90\x91\xe5\x81\x8f=%2m  \xe7\x9f\xad\xe5\x80\x92=%2m")
                    .arg(d, 0, 'f', 2).arg(fwd, 0, 'f', 2));
            }
        }
    };
    auto* calc_timer = new QTimer(this);
    calc_timer->setSingleShot(true);
    connect(calc_timer, &QTimer::timeout, this, updateParamView);
    auto schedule = [calc_timer](int ms) { calc_timer->start(ms); };
    connect(combo_path_mode_, &SegmentedControl::selectionChanged,
            this, [schedule](int){ schedule(0); });
    connect(chk_test_mode_, &SmoothToggle::toggled,
            this, [schedule](bool){ schedule(0); });
    connect(spin_push_length_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [schedule](double){ schedule(80); });
    connect(spin_shift_angle_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [schedule](double){ schedule(80); });
    connect(spin_blade_width_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [schedule](double){ schedule(80); });
    updateParamView();

    //=== 铲刀控制模式 ========================================================
    auto* levelSec = new GroupedSection(QString::fromUtf8(
        "\xe9\x93\xb2\xe5\x88\x80\xe6\x8e\xa7\xe5\x88\xb6\xe6\xa8\xa1\xe5\xbc\x8f (\xe7\xbb\x91\xe5\xae\x9a\xe5\x88\xb0\xe8\xb7\xaf\xe5\xbe\x84)"), inner);
    {
        combo_level_mode_ = new SegmentedControl({
            QString::fromUtf8("\xe6\x89\xbe\xe5\xb9\xb3 + \xe6\xa8\xaa\xe5\x9d\xa1"),
            QString::fromUtf8("\xe7\xba\xb5\xe5\x9d\xa1 + \xe6\xa8\xaa\xe5\x9d\xa1")});
        combo_level_mode_->setMinimumWidth(280);
        levelSec->addRow(new ParameterRow(QString::fromUtf8("\xe6\xa8\xa1\xe5\xbc\x8f"),
                                          combo_level_mode_, levelSec));

        spin_blade_angle_deg_ = makeSpin(-15.0, 15.0, 0.0, 0.1, 2);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_blade_angle_deg_)) s->setSafeRange(-8.0, 8.0);
        auto* crossRow = new ParameterRow(QString::fromUtf8("\xe6\xa8\xaa\xe5\x9d\xa1\xe8\xa7\x92\xe5\xba\xa6"),
                                          spin_blade_angle_deg_, levelSec);
        crossRow->setUnit(QString::fromUtf8("\xc2\xb0"));
        crossRow->setHint(QString::fromUtf8("0=\xe6\xb0\xb4\xe5\xb9\xb3  \xe6\xad\xa3/\xe8\xb4\x9f=\xe5\xb7\xa6/\xe5\x8f\xb3\xe5\x80\xbe"));
        levelSec->addRow(crossRow);
    }

    // 找平+横坡 面板
    flat3d_panel_ = new QWidget(levelSec);
    {
        auto* flatL = new QVBoxLayout(flat3d_panel_);
        flatL->setContentsMargins(0, 0, 0, 0);
        flatL->setSpacing(0);
        spin_target_level_height_ = makeSpin(-500.0, 9000.0, 0.0, 0.01, 3);
        if (auto* s = qobject_cast<SmartSpinBox*>(spin_target_level_height_)) s->setSafeRange(-2.0, 2.0);
        auto* tRow = new ParameterRow(QString::fromUtf8(
            "\xe7\x9b\xae\xe6\xa0\x87\xe9\xab\x98\xe7\xa8\x8b (ENU-Up)"),
            spin_target_level_height_, flat3d_panel_);
        tRow->setUnit("m");
        tRow->setHint(QString::fromUtf8(
            "\xe6\x89\x80\xe6\x9c\x89\xe6\x8e\xa8\xe5\x9c\x9f\xe6\xae\xb5\xe9\x93\xb2\xe5\x88\x80\xe5\x9d\x87\xe8\xb7\x9f\xe8\xb8\xaa\xe6\xad\xa4\xe7\xbb\x9d\xe5\xaf\xb9\xe9\xab\x98\xe7\xa8\x8b"));
        flatL->addWidget(tRow);
    }
    levelSec->addRow(flat3d_panel_);

    // 纵坡+横坡 面板
    slope3d_panel_ = new QWidget(levelSec);
    {
        auto* slopeL = new QVBoxLayout(slope3d_panel_);
        slopeL->setContentsMargins(0, 0, 0, 0);
        slopeL->setSpacing(0);
        spin_slope_start_height_ = makeSpin(-500.0, 9000.0, 0.0, 0.01, 3);
        auto* sRow = new ParameterRow(QString::fromUtf8(
            "\xe8\xb5\xb7\xe5\xa7\x8b\xe9\xab\x98\xe5\xba\xa6 (ENU-Up)"),
            spin_slope_start_height_, slope3d_panel_);
        sRow->setUnit("m");
        slopeL->addWidget(sRow);

        spin_slope_gradient_ = makeSpin(-20.0, 20.0, 0.0, 0.1, 2);
        auto* gRow = new ParameterRow(QString::fromUtf8(
            "\xe7\xba\xb5\xe5\x90\x91\xe5\x9d\xa1\xe5\xba\xa6"),
            spin_slope_gradient_, slope3d_panel_);
        gRow->setUnit("%");
        gRow->setHint(QString::fromUtf8(
            "\xe7\x9b\xae\xe6\xa0\x87\xe9\xab\x98\xe5\xba\xa6 = \xe8\xb5\xb7\xe5\xa7\x8b\xe9\xab\x98\xe5\xba\xa6 + \xe8\xa1\x8c\xe9\xa9\xb6\xe8\xb7\x9d\xe7\xa6\xbb \xc3\x97 \xe5\x9d\xa1\xe5\xba\xa6/100"));
        slopeL->addWidget(gRow);
    }
    slope3d_panel_->setVisible(false);
    levelSec->addRow(slope3d_panel_);

    connect(combo_level_mode_, &SegmentedControl::selectionChanged,
            [this](int idx) {
                flat3d_panel_->setVisible(idx == 0);
                slope3d_panel_->setVisible(idx == 1);
            });
    layout->addWidget(levelSec);

    //=== 辅助模块参数 ========================================================
    auto* auxSec = new GroupedSection(QString::fromUtf8(
        "\xe8\xbe\x85\xe5\x8a\xa9\xe6\xa8\xa1\xe5\x9d\x97\xe5\x8f\x82\xe6\x95\xb0"), inner);
    {
        auto addAux = [auxSec](const QString& label, QDoubleSpinBox* sb,
                               const QString& unit = QString()) {
            auto* r = new ParameterRow(label, sb, auxSec);
            if (!unit.isEmpty()) r->setUnit(unit);
            auxSec->addRow(r);
        };
        spin_tol_run_x_       = makeSpin(0, 5,    0.5,  0.1,  2);
        spin_tol_run_theta_   = makeSpin(0, 10,   2.0,  0.1,  1);
        spin_tol_mold_height_ = makeSpin(0, 1,    0.05, 0.01, 3);
        addAux(QString::fromUtf8("X \xe5\xae\xb9\xe5\xb7\xae"), spin_tol_run_x_, "m");
        addAux(QString::fromUtf8("\xce\xb8 \xe5\xae\xb9\xe5\xb7\xae"), spin_tol_run_theta_, QString::fromUtf8("\xc2\xb0"));
        addAux(QString::fromUtf8("\xe9\x93\xb2\xe5\x88\x80\xe9\xab\x98\xe5\xba\xa6\xe5\xae\xb9\xe5\xb7\xae"),
               spin_tol_mold_height_, "m");

        spin_speed_gain_risk_ = makeSpin(0, 2, 0.5,  0.1, 2);
        spin_speed_gain_mold_ = makeSpin(0, 2, 0.7,  0.1, 2);
        spin_mold_limit_      = makeSpin(0, 1, 0.1,  0.05,2);
        addAux(QString::fromUtf8("\xe9\xa3\x8e\xe9\x99\xa9\xe5\x87\x8f\xe9\x80\x9f"), spin_speed_gain_risk_);
        addAux(QString::fromUtf8("\xe9\x93\xb2\xe5\x88\x80\xe5\x87\x8f\xe9\x80\x9f"), spin_speed_gain_mold_);
        addAux(QString::fromUtf8("\xe9\x93\xb2\xe5\x88\x80\xe5\x8a\xa8\xe4\xbd\x9c\xe9\x99\x90"), spin_mold_limit_);

        spin_overload_trans_   = makeSpin(0, 5000, 1000, 100, 0);
        spin_overload_vehicle_ = makeSpin(0, 10,   0.5,  0.1, 2);
        spin_overload_angular_ = makeSpin(0, 100,  10,   1,   1);
        addAux(QString::fromUtf8("\xe5\x8f\x98\xe9\x80\x9f\xe7\xae\xb1\xe8\xbd\xac\xe9\x80\x9f\xe9\x99\x90"), spin_overload_trans_);
        addAux(QString::fromUtf8("\xe8\xbd\xa6\xe9\x80\x9f\xe9\x99\x90"), spin_overload_vehicle_, "m/s");
        addAux(QString::fromUtf8("\xe8\xa7\x92\xe9\x80\x9f\xe9\x99\x90"), spin_overload_angular_, QString::fromUtf8("\xc2\xb0/s"));
    }
    layout->addWidget(auxSec);

    //=== 操作按钮 ============================================================
    btn_send_plan_params_ = new IOSButton(QString::fromUtf8("\xe5\x8f\x91\xe9\x80\x81\xe5\x8f\x82\xe6\x95\xb0"),
                                          IOSButton::Success);
    btn_send_plan_params_->setMinimumHeight(40);
    btn_generate_path_ = new IOSButton(QString::fromUtf8("\xe2\x96\xb6 \xe7\x94\x9f\xe6\x88\x90\xe8\xb7\xaf\xe5\xbe\x84"),
                                       IOSButton::Primary);
    btn_generate_path_->setMinimumHeight(45);
    {
        auto* btnRow = new QHBoxLayout();
        btnRow->setContentsMargins(0, 0, 0, 0);
        btnRow->addWidget(btn_send_plan_params_);
        btnRow->addWidget(btn_generate_path_);
        auto* wrap = new QWidget(inner);
        wrap->setLayout(btnRow);
        layout->addWidget(wrap);
    }

    btn_start_exec_ = new IOSButton(QString::fromUtf8("\xe2\x96\xb6 \xe5\xbc\x80\xe5\xa7\x8b\xe6\x89\xa7\xe8\xa1\x8c"),
                                    IOSButton::Success);
    btn_start_exec_->setMinimumHeight(45);
    btn_pause_exec_ = new IOSButton(QString::fromUtf8("\xe2\x8f\xb8 \xe6\x9a\x82\xe5\x81\x9c"),
                                    IOSButton::Warn);
    btn_pause_exec_->setMinimumHeight(45);
    btn_pause_exec_->setEnabled(false);
    {
        auto* execBtnRow = new QHBoxLayout();
        execBtnRow->setContentsMargins(0, 0, 0, 0);
        execBtnRow->addWidget(btn_start_exec_);
        execBtnRow->addWidget(btn_pause_exec_);
        execBtnRow->addWidget(btn_path_stop_);
        auto* wrap = new QWidget(inner);
        wrap->setLayout(execBtnRow);
        layout->addWidget(wrap);
    }

    label_plan_params_ack_ = new QLabel("");
    label_plan_params_ack_->setFont(MONO_FONT);
    layout->addWidget(label_plan_params_ack_);
    layout->addStretch();

    scroll->setWidget(inner);
    auto* tabL = new QVBoxLayout(tab);
    tabL->setContentsMargins(0, 0, 0, 0);
    tabL->addWidget(scroll);
    tabs->addTab(tab, "\U0001F5FA 规划参数");
}

//==============================================================================
// Tab6: 车辆配置
//==============================================================================
void ManualControlWidget::setupTab6_VehicleConfig(QTabWidget* tabs) {
    // 阶段三 · 布局重构: GroupedSection + ParameterRow + StickyFooter.
    auto* tab    = new QWidget();
    auto* outer  = new QVBoxLayout(tab);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    auto* scroll = new RubberScrollArea(tab);
    auto* inner  = new QWidget();
    inner->setStyleSheet(QStringLiteral("QWidget{background:%1;}")
                         .arg(Theme::hex(Theme::bgWindow)));
    auto* layout = new QVBoxLayout(inner);
    layout->setContentsMargins(Spacing::panelPadding, Spacing::panelPadding,
                               Spacing::panelPadding, 8);
    layout->setSpacing(Spacing::sectionGap);

    // === 预设管理 ===
    auto* presetSec = new GroupedSection(QString::fromUtf8(
        "\xe9\xa2\x84\xe8\xae\xbe\xe7\xae\xa1\xe7\x90\x86"), inner);
    {
        combo_vehicle_preset_ = new QComboBox(presetSec);
        combo_vehicle_preset_->setMinimumHeight(32);
        combo_vehicle_preset_->setMinimumWidth(220);
        presetSec->addRow(new ParameterRow(QString::fromUtf8(
            "\xe9\xa2\x84\xe8\xae\xbe"), combo_vehicle_preset_, presetSec));

        // 操作按钮放一行 (作为容器嵌入分组)
        btn_save_preset_   = new IOSButton(QString::fromUtf8(
            "\xe5\x8f\xa6\xe5\xad\x98\xe4\xb8\xba\xe9\xa2\x84\xe8\xae\xbe"), IOSButton::Primary);
        btn_delete_preset_ = new IOSButton(QString::fromUtf8(
            "\xe5\x88\xa0\xe9\x99\xa4"), IOSButton::Danger);
        auto* btnRow = new QWidget(presetSec);
        auto* btnL   = new QHBoxLayout(btnRow);
        btnL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        btnL->setSpacing(8);
        btnL->addStretch();
        btnL->addWidget(btn_save_preset_);
        btnL->addWidget(btn_delete_preset_);
        presetSec->addRow(btnRow);
    }
    layout->addWidget(presetSec);

    // === 配置文件 ===
    auto* fileSec = new GroupedSection(QString::fromUtf8(
        "\xe9\x85\x8d\xe7\xbd\xae\xe6\x96\x87\xe4\xbb\xb6"), inner);
    {
        edit_config_path_ = new QLineEdit("./params/default.txt", fileSec);
        edit_config_path_->setMinimumHeight(32);
        fileSec->addRow(new ParameterRow(QString::fromUtf8(
            "\xe6\x96\x87\xe4\xbb\xb6\xe8\xb7\xaf\xe5\xbe\x84"),
            edit_config_path_, fileSec));

        btn_save_config_   = new IOSButton(QString::fromUtf8(
            "\xe4\xbf\x9d\xe5\xad\x98\xe9\x85\x8d\xe7\xbd\xae"), IOSButton::Primary);
        btn_load_config_   = new IOSButton(QString::fromUtf8(
            "\xe5\x8a\xa0\xe8\xbd\xbd\xe9\x85\x8d\xe7\xbd\xae"), IOSButton::Success);
        // btn_load_and_send_ 创建在 StickyFooter 里 (主操作)
        auto* btnRow = new QWidget(fileSec);
        auto* btnL   = new QHBoxLayout(btnRow);
        btnL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        btnL->setSpacing(8);
        btnL->addStretch();
        btnL->addWidget(btn_load_config_);
        btnL->addWidget(btn_save_config_);
        fileSec->addRow(btnRow);
    }
    layout->addWidget(fileSec);

    // === 历史记录 ===
    auto* histSec = new GroupedSection(QString::fromUtf8(
        "\xe5\x8e\x86\xe5\x8f\xb2\xe8\xae\xb0\xe5\xbd\x95"), inner);
    {
        combo_history_ = new QComboBox(histSec);
        combo_history_->setMinimumHeight(32);
        combo_history_->setMinimumWidth(280);
        histSec->addRow(new ParameterRow(QString::fromUtf8(
            "\xe6\x97\xb6\xe9\x97\xb4\xe6\x88\xb3"),
            combo_history_, histSec));

        btn_restore_history_ = new IOSButton(QString::fromUtf8(
            "\xe6\x81\xa2\xe5\xa4\x8d"), IOSButton::Success);
        btn_view_changelog_  = new IOSButton(QString::fromUtf8(
            "\xe5\x8f\x98\xe6\x9b\xb4\xe6\x97\xa5\xe5\xbf\x97"), IOSButton::Secondary);
        auto* btnRow = new QWidget(histSec);
        auto* btnL   = new QHBoxLayout(btnRow);
        btnL->setContentsMargins(Spacing::panelPadding, 4, Spacing::panelPadding, 8);
        btnL->setSpacing(8);
        btnL->addStretch();
        btnL->addWidget(btn_view_changelog_);
        btnL->addWidget(btn_restore_history_);
        histSec->addRow(btnRow);
    }
    layout->addWidget(histSec);

    // 说明
    auto* note = new QLabel(QString::fromUtf8(
        "\xe8\xaf\xb4\xe6\x98\x8e: \xe4\xbf\x9d\xe5\xad\x98\xe6\x97\xb6\xe8\x87\xaa\xe5\x8a\xa8\xe5\x88\x9b\xe5\xbb\xba\xe5\x8e\x86\xe5\x8f\xb2\xe5\xa4\x87\xe4\xbb\xbd\xe5\x92\x8c\xe5\x8f\x98\xe6\x9b\xb4\xe6\x97\xa5\xe5\xbf\x97\xe3\x80\x82\xe9\xa2\x84\xe8\xae\xbe\xe5\x8f\xaf\xe8\xb5\xb7\xe5\x90\x8d\xe4\xbf\x9d\xe5\xad\x98\xe5\xa4\x9a\xe5\xa5\x97\xe5\x8f\x82\xe6\x95\xb0\xe3\x80\x82"),
        inner);
    note->setWordWrap(true);
    note->setStyleSheet("color:#AEAEB2;padding:6px 12px;");
    layout->addWidget(note);
    layout->addStretch();

    scroll->setWidget(inner);
    outer->addWidget(scroll, 1);

    // === StickyFooter — "加载并发送" 主操作 ===
    auto* footer = new StickyFooter(QString::fromUtf8(
        "\xe5\xba\x94\xe7\x94\xa8\xe9\x85\x8d\xe7\xbd\xae (\xe5\x8a\xa0\xe8\xbd\xbd + \xe5\x8f\x91\xe9\x80\x81)"),
        tab);
    footer->button()->setVariant(IOSButton::Warn);
    btn_load_and_send_ = footer->button();   // 兼容旧引用
    outer->addWidget(footer);

    tabs->addTab(tab, "\U0001F4CB 车辆配置");

    refreshPresetList();
    refreshHistoryList();
}

//==============================================================================
// Tab7: 数据录制
//==============================================================================
void ManualControlWidget::setupTab7_DataRecord(QTabWidget* tabs) {
    auto* tab = new QWidget();
    auto* layout = new QVBoxLayout(tab);

    // 录制控制 + 保存路径
    auto* cG = makeGroup("\u5F55\u5236\u63A7\u5236");
    auto* cL = new QVBoxLayout(cG);
    auto* btnRow = new QHBoxLayout();
    // Tab7 录制按钮 — Success/Danger 语义色 + 按压反馈
    btn_start_record_ = new IOSButton("\u25CF \u5F00\u59CB\u5F55\u5236", IOSButton::Success);
    btn_start_record_->setMinimumHeight(45);
    btn_stop_record_ = new IOSButton("\u25A0 \u505C\u6B62\u5F55\u5236", IOSButton::Danger);
    btn_stop_record_->setMinimumHeight(45);
    btn_stop_record_->setEnabled(false);
    label_record_status_ = new QLabel("\u72B6\u6001: \u672A\u5F55\u5236"); label_record_status_->setFont(SIDE_FONT);
    btnRow->addWidget(btn_start_record_); btnRow->addWidget(btn_stop_record_); btnRow->addWidget(label_record_status_);
    cL->addLayout(btnRow);
    // 保存路径
    auto* pathRow = new QHBoxLayout();
    pathRow->addWidget(new QLabel("\u4FDD\u5B58\u8DEF\u5F84:"));
    edit_record_path_ = new QLineEdit(QDir::homePath() + "/rosbag_records", tab);
    edit_record_path_->setFont(MONO_FONT);
    auto* btnBrowse = new IOSButton("\u6D4F\u89C8...", IOSButton::Secondary);
    connect(btnBrowse, &QPushButton::clicked, [this]{
        QString dir = QFileDialog::getExistingDirectory(this, "\u9009\u62E9\u5F55\u5236\u76EE\u5F55", edit_record_path_->text());
        if (!dir.isEmpty()) edit_record_path_->setText(dir);
    });
    pathRow->addWidget(edit_record_path_, 1); pathRow->addWidget(btnBrowse);
    cL->addLayout(pathRow);
    layout->addWidget(cG);

    // 话题列表 (勾选)
    auto* tG = makeGroup("\u5F55\u5236\u8BDD\u9898 (\u52FE\u9009\u8981\u5F55\u5236\u7684, \u4E0D\u52FE\u9009\u5219\u5F55\u5236\u5168\u90E8)");
    auto* tL = new QVBoxLayout(tG);
    record_topic_list_ = new QListWidget(tab);
    record_topic_list_->setFont(MONO_FONT);
    record_topic_list_->setStyleSheet("QListWidget{background:white;border:1px solid #E0E0E5;border-radius:8px;}"
        "QListWidget::item{padding:4px 8px;border-bottom:1px solid #F9FAFB;}"
        "QListWidget::item:selected{background:#E6F0FF;}");
    // 预设常用话题 (默认全部勾选)
    QStringList defaultTopics = {
        "/LLA", "/Angle_Heading", "/AHRS_IMU", "/RTK",
        "/Engine_Speed_Actual", "/Vehicle_Speed_Vel", "/Transmission_Speed_Actual",
        "/Coolant_Temperature", "/Gear_Position", "/Fuel_Level",
        "/decision/walk_state", "/Decision_Status",
        "/control/V_right", "/control/V_left", "/control/terminal_flag",
        "/U_Lever1", "/U_Lever_Moldboard",
        "/occupancy_grid", "/Path", "/Direction"
    };
    for (const auto& t : defaultTopics) {
        auto* item = new QListWidgetItem(t, record_topic_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }
    tL->addWidget(record_topic_list_, 1);

    // 添加自定义话题 + 全选/全不选
    auto* addRow = new QHBoxLayout();
    edit_custom_topic_ = new QLineEdit(tab);
    edit_custom_topic_->setPlaceholderText("\u8F93\u5165\u8BDD\u9898\u540D (\u5982 /my_topic)");
    auto* btnAdd = new IOSButton("\u6DFB\u52A0", IOSButton::Primary);
    connect(btnAdd, &QPushButton::clicked, [this]{
        QString topic = edit_custom_topic_->text().trimmed();
        if (topic.isEmpty() || !topic.startsWith("/")) return;
        // 检查重复
        for (int i = 0; i < record_topic_list_->count(); ++i) {
            if (record_topic_list_->item(i)->text() == topic) return;
        }
        auto* item = new QListWidgetItem(topic, record_topic_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
        edit_custom_topic_->clear();
    });
    connect(edit_custom_topic_, &QLineEdit::returnPressed, [btnAdd]{ btnAdd->click(); });
    auto* btnSelectAll = new IOSButton("\u5168\u9009", IOSButton::Secondary);
    connect(btnSelectAll, &QPushButton::clicked, [this]{
        for (int i = 0; i < record_topic_list_->count(); ++i)
            record_topic_list_->item(i)->setCheckState(Qt::Checked);
    });
    auto* btnDeselectAll = new IOSButton("\u5168\u4E0D\u9009", IOSButton::Secondary);
    connect(btnDeselectAll, &QPushButton::clicked, [this]{
        for (int i = 0; i < record_topic_list_->count(); ++i)
            record_topic_list_->item(i)->setCheckState(Qt::Unchecked);
    });
    auto* btnRemove = new IOSButton("\u5220\u9664\u9009\u4E2D", IOSButton::Danger);
    connect(btnRemove, &QPushButton::clicked, [this]{
        auto* cur = record_topic_list_->currentItem();
        if (cur) delete cur;
    });
    addRow->addWidget(edit_custom_topic_, 1); addRow->addWidget(btnAdd);
    addRow->addWidget(btnSelectAll); addRow->addWidget(btnDeselectAll); addRow->addWidget(btnRemove);
    tL->addLayout(addRow);
    layout->addWidget(tG, 1);
    tabs->addTab(tab, "\U0001F3A5 \u6570\u636E\u5F55\u5236");
}

//==============================================================================
// Tab8: 通信诊断
//==============================================================================
void ManualControlWidget::setupTab8_CommDiag(QTabWidget* tabs) {
    auto* tab = new QWidget();
    auto* scroll = new QScrollArea(tab);
    scroll->setWidgetResizable(true);
    auto* inner = new QWidget();
    diag_layout_ = new QVBoxLayout(inner);

    // 硬件在线状态汇总 (基于话题是否有数据判断)
    auto* hwG = makeGroup("\u2705 \u786C\u4EF6\u72B6\u6001");
    auto* hwL = new QHBoxLayout(hwG);
    // 4 个 iOS 状态胶囊: 左圆点(绿=在线/红=离线) + 标签 + 值文本
    label_hw_rtk_    = new StatusPill("RTK");
    label_hw_imu_    = new StatusPill("IMU");
    label_hw_engine_ = new StatusPill("ECU");
    label_hw_can_    = new StatusPill("CAN");
    hwL->addWidget(label_hw_rtk_); hwL->addWidget(label_hw_imu_);
    hwL->addWidget(label_hw_engine_); hwL->addWidget(label_hw_can_);
    diag_layout_->addWidget(hwG);

    // 标题行
    auto* hdr = new QHBoxLayout();
    auto* h1 = new QLabel("<b>\u8BDD\u9898</b>"); h1->setFixedWidth(260); hdr->addWidget(h1);
    auto* h2 = new QLabel("<b>\u9891\u7387</b>"); h2->setFixedWidth(60); hdr->addWidget(h2);
    auto* h3 = new QLabel("<b>\u72B6\u6001</b>"); h3->setFixedWidth(50); hdr->addWidget(h3);
    auto* h4 = new QLabel("<b>\u6700\u65B0\u503C</b>"); hdr->addWidget(h4);
    diag_layout_->addLayout(hdr);

    // 感知层
    auto* grpSensor = new QLabel("\u2501\u2501 \u611F\u77E5\u5C42 \u2501\u2501"); grpSensor->setStyleSheet("color:#007AFF;font-weight:bold;padding:4px 0;");
    diag_layout_->addWidget(grpSensor);
    for (const auto& t : {"/LLA", "/Angle_Heading", "/AHRS_IMU", "/RTK",
                          "/Navigate_location", "/occupancy_grid",
                          "/risk_state", "/IMU_Pitch", "/IMU_Roll",
                          "/moldboard/actual_height_left", "/moldboard/actual_height_right",
                          "/moldboard/imu_roll"})
        addDiagTopic(QString(t));

    // 规控层
    auto* grpCtrl = new QLabel("\u2501\u2501 \u89C4\u63A7\u5C42 \u2501\u2501"); grpCtrl->setStyleSheet("color:#FF9500;font-weight:bold;padding:4px 0;");
    diag_layout_->addWidget(grpCtrl);
    for (const auto& t : {"/decision/walk_state", "/decision/exec_state",
                          "/decision/waypoint_index", "/decision/main_switch",
                          "/decision/moldboard_control_flag",
                          "/Decision_Status", "/control_speed_gain",
                          "/control/V_right", "/control/V_left",
                          "/control/terminal_flag", "/control/cmd_vel",
                          "/control/moldboard_debug"})
        addDiagTopic(QString(t));

    // 执行层
    auto* grpExec = new QLabel("\u2501\u2501 \u6267\u884C\u5C42 \u2501\u2501"); grpExec->setStyleSheet("color:#34C759;font-weight:bold;padding:4px 0;");
    diag_layout_->addWidget(grpExec);
    for (const auto& t : {"/U_Lever1", "/U_Lever_Moldboard", "/Mode_Switch", "/Path"})
        addDiagTopic(QString(t));

    // 整车状态
    auto* grpVeh = new QLabel("\u2501\u2501 \u6574\u8F66 \u2501\u2501"); grpVeh->setStyleSheet("color:#007AFF;font-weight:bold;padding:4px 0;");
    diag_layout_->addWidget(grpVeh);
    for (const auto& t : {"/Engine_Speed_Actual", "/Vehicle_Speed_Vel",
                          "/Transmission_Speed_Actual", "/Coolant_Temperature",
                          "/Gear_Position", "/Gear_Direction",
                          "/Manual_Auto_Switch", "/Mold_OverLoad_Status"})
        addDiagTopic(QString(t));

    diag_layout_->addStretch();
    scroll->setWidget(inner);
    auto* tabL = new QVBoxLayout(tab);
    tabL->setContentsMargins(0,0,0,0);
    tabL->addWidget(scroll);
    tabs->addTab(tab, "\U0001F4E1 \u901A\u4FE1\u8BCA\u65AD");
}

void ManualControlWidget::addDiagTopic(const QString& name) {
    TopicDiag d;
    auto* row = new QHBoxLayout();
    d.name_label = new QLabel(name); d.name_label->setFixedWidth(260); d.name_label->setFont(MONO_FONT);
    d.freq_label = new QLabel("0 Hz"); d.freq_label->setFixedWidth(60); d.freq_label->setFont(MONO_FONT);
    d.status_label = new QLabel("等待"); d.status_label->setFixedWidth(50); d.status_label->setFont(MONO_FONT);
    d.status_label->setStyleSheet("color:#AEAEB2;");
    d.value_label = new QLabel("--"); d.value_label->setFont(MONO_FONT);
    d.value_label->setStyleSheet("color:#8E8E93;");
    row->addWidget(d.name_label); row->addWidget(d.freq_label);
    row->addWidget(d.status_label); row->addWidget(d.value_label);
    diag_layout_->addLayout(row);
    topic_diags_[name] = d;
}

void ManualControlWidget::recordTopicValue(const QString& topic, const QString& value) {
    if (topic_diags_.contains(topic)) {
        topic_diags_[topic].count++;
        topic_diags_[topic].last_time_ms = diag_elapsed_.elapsed();
        topic_diags_[topic].last_value = value;
    }
}

//==============================================================================
// ROS 发布/订阅
//==============================================================================
void ManualControlWidget::setupPublishers() {
    pub_lever1_ = nh_.advertise<std_msgs::Int16MultiArray>("/U_Lever1", 1);
    pub_lever_moldboard_ = nh_.advertise<std_msgs::Int16MultiArray>("/U_Lever_Moldboard", 1);
    pub_moldboard_flag_direct_ = nh_.advertise<std_msgs::Int16>("/Moldboard_Control_Flag_Plan", 1);
    pub_mode_switch_ = nh_.advertise<std_msgs::Int32>("/Mode_Switch", 1);
    pub_engine_speed_ = nh_.advertise<std_msgs::Int32>("/U_Engine_Speed", 1);
    pub_brake_ = nh_.advertise<std_msgs::Int32>("/U_Brake", 1);
    pub_main_switch_ = nh_.advertise<std_msgs::Float64>("/Main_Switch", 1);
    pub_main_switch_ctrl_ = nh_.advertise<std_msgs::Float64>("/decision/main_switch", 1);
    pub_detection_ = nh_.advertise<std_msgs::Float64>("/Detection_Altitude_Completed", 1);
    pub_walk_state_ = nh_.advertise<std_msgs::Float64>("/decision/walk_state", 1);
    pub_terminal_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/terminal", 1);
    pub_reference_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/reference", 1);
    pub_mold_ctrl_flag_ = nh_.advertise<std_msgs::Float64>("/decision/moldboard_control_flag", 1);
    pub_ref_height_ = nh_.advertise<std_msgs::Float64>("/decision/ref_height_middle_moldboard", 1);
    pub_ref_angle_ = nh_.advertise<std_msgs::Float64>("/decision/ref_angle", 1);
    pub_set_blade_origin_ = nh_.advertise<geometry_msgs::Point>("/decision/set_blade_origin", 1);  // [Issue#7]
    pub_walk_params_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/params", 1);
    pub_mold_params_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/moldboard_params", 1);
    pub_test_config_ = nh_.advertise<std_msgs::Float64MultiArray>("/decision/test_config", 1);
    pub_test_script_ = nh_.advertise<std_msgs::String>("/decision/test_script", 1);
    pub_full_path_preview_ = nh_.advertise<std_msgs::Float64MultiArray>("/decision/full_path", 1, true);
    pub_regen_path_  = nh_.advertise<std_msgs::Float64>("/decision/regenerate_path", 1);
    pub_path_mode_   = nh_.advertise<std_msgs::Float64>("/decision/path_mode", 1);
    pub_path_params_ = nh_.advertise<std_msgs::Float64MultiArray>("/decision/path_params", 1);
    pub_path_stop_ = nh_.advertise<std_msgs::Float64>("/decision/emergency_stop", 1);

    // 执行状态订阅
    sub_exec_state_ = nh_.subscribe<std_msgs::Float64>("/decision/exec_state", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                static const char* names[] = {"\u7A7A\u95F2","\u65CB\u8F6C\u4E2D","\u76F4\u884C\u4E2D",
                    "\u5230\u4F4D","\u5B8C\u6210","\u7D27\u6025\u505C","\u8FC7\u8F7D\u540E\u9000"};
                static const char* colors[] = {"#AEAEB2","#FF9500","#007AFF",
                    "#34C759","#34C759","#FF3B30","#FF9500"};
                int s = static_cast<int>(msg->data);
                if (s < 0 || s > 6) s = 0;
                label_exec_state_->setText(names[s]);
                label_exec_state_->setStyleSheet(
                    QString("color:%1;font-size:14px;font-weight:bold;padding:4px 8px;"
                            "background:rgba(142,142,147,0.1);border-radius:6px;").arg(colors[s]));
                recordTopicValue("/decision/exec_state", names[s]);
            }));
    sub_wp_index_ = nh_.subscribe<std_msgs::Float64>("/decision/waypoint_index", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                label_wp_progress_->setText(QString("WP: %1").arg(static_cast<int>(msg->data)));
                recordTopicValue("/decision/waypoint_index", QString::number(static_cast<int>(msg->data)));
            }));
}

void ManualControlWidget::setupSubscribers() {
    sub_lla_ = nh_.subscribe("/LLA",1,&ManualControlWidget::llaCallback,this);
    sub_angle_ = nh_.subscribe("/Angle_Heading",1,&ManualControlWidget::angleCallback,this);
    sub_imu_ = nh_.subscribe("/AHRS_IMU",1,&ManualControlWidget::imuCallback,this);
    sub_walk_state_ = nh_.subscribe("/decision/walk_state",1,&ManualControlWidget::walkStateCallback,this);
    sub_decision_status_ = nh_.subscribe("/Decision_Status",1,&ManualControlWidget::decisionStatusCallback,this);
    sub_moldboard_actual_ = nh_.subscribe("/Bulldozer_Moldboard_Actual",1,&ManualControlWidget::moldboardActualCallback,this);
    sub_v_right_ = nh_.subscribe("/control/V_right",1,&ManualControlWidget::vRightCallback,this);
    sub_v_left_ = nh_.subscribe("/control/V_left",1,&ManualControlWidget::vLeftCallback,this);
    sub_terminal_flag_ = nh_.subscribe("/control/terminal_flag",1,&ManualControlWidget::terminalFlagCallback,this);
    sub_moldboard_debug_ = nh_.subscribe("/control/moldboard_debug",1,&ManualControlWidget::moldboardDebugCallback,this);
    sub_engine_speed_ = nh_.subscribe("/Engine_Speed_Actual",1,&ManualControlWidget::engineSpeedCallback,this);
    sub_engine_torque_ = nh_.subscribe("/Engine_Torque_Actual",1,&ManualControlWidget::engineTorqueCallback,this);
    sub_trans_speed_ = nh_.subscribe("/Transmission_Speed_Actual",1,&ManualControlWidget::transSpeedCallback,this);
    sub_vehicle_speed_ = nh_.subscribe("/Vehicle_Speed_Vel",1,&ManualControlWidget::vehicleSpeedCallback,this);
    sub_output_current_ = nh_.subscribe("/OUTPUT_CURRENT",1,&ManualControlWidget::outputCurrentCallback,this);
    sub_imu_pitch_ = nh_.subscribe("/IMU_Pitch",1,&ManualControlWidget::imuPitchCallback,this);
    sub_imu_roll_ = nh_.subscribe("/IMU_Roll",1,&ManualControlWidget::imuRollCallback,this);
    // [Fix-v19] 订阅 moldboard_controller 发的铲刀 IMU 横滚角 (由 /AHRS_IMU 经 moldboard 转发)
    //   用于 autotune 角度通道反馈 + GUI 铲刀 Roll 显示
    //   (旧 /IMU_Roll 来自旧款 ACC IMU, 硬件不存在, 值永远是 0)
    sub_mold_imu_roll_ = nh_.subscribe<std_msgs::Float64>("/moldboard/imu_roll", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& m){ mold_imu_roll_deg_ = m->data; }));
    // 栅格地图尺寸 (用于Tab5显示)
    sub_map_info_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/occupancy_grid", 1,
        boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)>(
            [this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
                map_rows_ = msg->info.height;
                map_cols_ = msg->info.width;
                map_res_ = msg->info.resolution;
            }));
    // 新增: 车辆状态订阅 (含 recordTopicValue 给 Tab8 诊断)
    sub_coolant_temp_ = nh_.subscribe<std_msgs::Float64>("/Coolant_Temperature",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            coolant_temp_=m->data; recordTopicValue("/Coolant_Temperature", QString("%1\u00B0C").arg(m->data,0,'f',1));}));
    sub_engine_hours_ = nh_.subscribe<std_msgs::Float64>("/Engine_Total_Hours",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){engine_hours_=m->data;}));
    sub_gear_pos_ = nh_.subscribe<std_msgs::UInt8>("/Gear_Position",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){
            gear_pos_=m->data; recordTopicValue("/Gear_Position", QString::number(m->data));}));
    sub_gear_dir_ = nh_.subscribe<std_msgs::UInt8>("/Gear_Direction",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){
            gear_dir_=m->data;
            const char* dn[]={"N","F","R"};
            recordTopicValue("/Gear_Direction", (m->data<=2)?dn[m->data]:"?");}));
    sub_hand_brake_ = nh_.subscribe<std_msgs::UInt8>("/Hand_Brake",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){hand_brake_=m->data;}));
    sub_hydraulic_lock_ = nh_.subscribe<std_msgs::UInt8>("/Hydraulic_Lock",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){hydraulic_lock_=m->data;}));
    sub_brake_valve_ = nh_.subscribe<std_msgs::UInt8>("/Brake_Valve_Open",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){brake_valve_=m->data;}));
    sub_fuel_level_ = nh_.subscribe<std_msgs::UInt8>("/Fuel_Level",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){fuel_level_=m->data;}));
    sub_fuel_total_ = nh_.subscribe<std_msgs::Float64>("/Fuel_Total",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){fuel_total_=m->data;}));
    sub_manual_auto_ = nh_.subscribe<std_msgs::UInt8>("/Manual_Auto_Switch",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){
            manual_auto_=m->data; recordTopicValue("/Manual_Auto_Switch", m->data?"\u81EA\u52A8":"\u624B\u52A8");}));
    sub_handle_turn_ = nh_.subscribe<std_msgs::UInt8>("/Handle_Turn_Value",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){handle_turn_val_=m->data;}));
    sub_sys_error_ = nh_.subscribe<std_msgs::UInt32>("/systemErrorCode",1,
        boost::function<void(const std_msgs::UInt32::ConstPtr&)>([this](const std_msgs::UInt32::ConstPtr& m){sys_error_code_=m->data;}));
    sub_mil_light_ = nh_.subscribe<std_msgs::UInt8>("/MIL_Light",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){mil_light_=m->data;}));
    sub_red_stop_ = nh_.subscribe<std_msgs::UInt8>("/Red_Stop_Light",1,
        boost::function<void(const std_msgs::UInt8::ConstPtr&)>([this](const std_msgs::UInt8::ConstPtr& m){red_stop_=m->data;}));

    // 纯诊断订阅: 新增话题 (只为 Tab8 显示频率+值, 不影响控制逻辑)
    nh_.subscribe<geometry_msgs::Point>("/Navigate_location",1,
        boost::function<void(const geometry_msgs::Point::ConstPtr&)>([this](const geometry_msgs::Point::ConstPtr& m){
            recordTopicValue("/Navigate_location", QString("x=%1 y=%2").arg(m->x,0,'f',2).arg(m->y,0,'f',2));}));
    nh_.subscribe<std_msgs::Int8MultiArray>("/risk_state",1,
        boost::function<void(const std_msgs::Int8MultiArray::ConstPtr&)>([this](const std_msgs::Int8MultiArray::ConstPtr& m){
            QString v; for(int i=0;i<(int)m->data.size()&&i<4;i++){ risk_state_[i]=m->data[i]; v+=QString::number(m->data[i])+" "; }
            risk_state_last_ms_ = diag_elapsed_.elapsed();
            recordTopicValue("/risk_state", v.trimmed());}));
    nh_.subscribe<std_msgs::Float64>("/moldboard/actual_height_left",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/moldboard/actual_height_left", QString("%1mm").arg(m->data,0,'f',1));}));
    nh_.subscribe<std_msgs::Float64>("/moldboard/actual_height_right",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/moldboard/actual_height_right", QString("%1mm").arg(m->data,0,'f',1));}));
    nh_.subscribe<std_msgs::Float64>("/moldboard/imu_roll",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/moldboard/imu_roll", QString("%1\u00B0").arg(m->data,0,'f',1));}));
    nh_.subscribe<std_msgs::Float64>("/decision/main_switch",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/decision/main_switch", m->data>0.5?"ON":"OFF");}));
    nh_.subscribe<std_msgs::Float64>("/decision/moldboard_control_flag",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/decision/moldboard_control_flag", m->data>0.5?"ON":"OFF");}));
    nh_.subscribe<std_msgs::Float64>("/control_speed_gain",1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>([this](const std_msgs::Float64::ConstPtr& m){
            recordTopicValue("/control_speed_gain", QString("%1").arg(m->data,0,'f',2));}));
    nh_.subscribe<geometry_msgs::Twist>("/control/cmd_vel",1,
        boost::function<void(const geometry_msgs::Twist::ConstPtr&)>([this](const geometry_msgs::Twist::ConstPtr& m){
            recordTopicValue("/control/cmd_vel", QString("v=%1 w=%2").arg(m->linear.x,0,'f',2).arg(m->angular.z,0,'f',2));}));
    nh_.subscribe<std_msgs::Int16>("/Mold_OverLoad_Status",1,
        boost::function<void(const std_msgs::Int16::ConstPtr&)>([this](const std_msgs::Int16::ConstPtr& m){
            recordTopicValue("/Mold_OverLoad_Status", m->data?"OVERLOAD":"OK");}));
}

//==============================================================================
// 信号连接
//==============================================================================
void ManualControlWidget::setupConnections() {
    auto press = [](bool& f){ return [&f]{ f=true; }; };
    auto release = [](bool& f){ return [&f]{ f=false; }; };
    // Tab1
    connect(btn_forward_,&QPushButton::pressed,press(forward_pressed_));  connect(btn_forward_,&QPushButton::released,release(forward_pressed_));
    connect(btn_backward_,&QPushButton::pressed,press(backward_pressed_));connect(btn_backward_,&QPushButton::released,release(backward_pressed_));
    connect(btn_left_,&QPushButton::pressed,press(left_pressed_));        connect(btn_left_,&QPushButton::released,release(left_pressed_));
    connect(btn_right_,&QPushButton::pressed,press(right_pressed_));      connect(btn_right_,&QPushButton::released,release(right_pressed_));
    connect(btn_gear_up_,&QPushButton::pressed,press(gear_up_pressed_));  connect(btn_gear_up_,&QPushButton::released,release(gear_up_pressed_));
    connect(btn_gear_down_,&QPushButton::pressed,press(gear_down_pressed_));connect(btn_gear_down_,&QPushButton::released,release(gear_down_pressed_));
    connect(btn_blade_up_,&QPushButton::pressed,press(blade_up_pressed_));connect(btn_blade_up_,&QPushButton::released,release(blade_up_pressed_));
    connect(btn_blade_down_,&QPushButton::pressed,press(blade_down_pressed_));connect(btn_blade_down_,&QPushButton::released,release(blade_down_pressed_));
    connect(btn_blade_tilt_left_,&QPushButton::pressed,press(blade_tilt_l_pressed_));connect(btn_blade_tilt_left_,&QPushButton::released,release(blade_tilt_l_pressed_));
    connect(btn_blade_tilt_right_,&QPushButton::pressed,press(blade_tilt_r_pressed_));connect(btn_blade_tilt_right_,&QPushButton::released,release(blade_tilt_r_pressed_));
    connect(btn_unlock_,&QPushButton::clicked,[this]{ locked_=false; unlock_pressed_=true; btn_unlock_->setStyleSheet(STY_GREEN); btn_stop_->setStyleSheet(STY_RED); });
    connect(btn_stop_,&QPushButton::clicked,[this]{ locked_=true; unlock_pressed_=false; forward_pressed_=backward_pressed_=left_pressed_=right_pressed_=false; gear_up_pressed_=gear_down_pressed_=false; blade_up_pressed_=blade_down_pressed_=blade_tilt_l_pressed_=blade_tilt_r_pressed_=false; brake_pressed_=false; btn_stop_->setStyleSheet("QPushButton{background:#D32F2F;color:white;border-radius:6px;font-weight:bold;}"); btn_unlock_->setStyleSheet(STY_GRAY); });
    connect(btn_brake_,&QPushButton::pressed,[this]{ brake_pressed_=true; });
    connect(btn_brake_,&QPushButton::released,[this]{ brake_pressed_=false; });
    connect(slider_steering_,&QSlider::valueChanged,[this](int v){label_steering_val_->setText(QString::number(v));});
    connect(slider_blade_speed_,&QSlider::valueChanged,[this](int v){label_blade_speed_val_->setText(QString::number(v));});
    connect(chk_blade_enable_,&QCheckBox::toggled,[this](bool){ /* 持续发送在 publishLever1() 定时器里 */ });
    // SmoothToggle 自己管外观, 这里只做业务逻辑 (比如发 CAN 帧在 publishLever1 里按 isChecked 判断).
    connect(btn_unmanned_mode_,&SmoothToggle::toggled,[this](bool on){
        // 视觉反馈已由 unmanStatus label 和 SmoothToggle 动画处理, 这里保留空 slot 便于扩展
        (void)on;
    });

    // Tab2
    connect(mode_group_, &SegmentedControl::selectionChanged, [this](int id){
        current_mode_=static_cast<NodeTestMode>(id); node_walk_mode_=0;
        group_auto_->setVisible(current_mode_==MODE_AUTO); group_walk_->setVisible(current_mode_==MODE_WALK); group_blade_->setVisible(current_mode_==MODE_BLADE); group_blade_origin_->setVisible(current_mode_==MODE_BLADE);  // [Issue#7]
    });
    connect(btn_node_main_switch_,&SmoothToggle::toggled,[this](bool on){
        node_main_switch_on_ = on;
        log_output_->append(QString("[主开关] %1").arg(on?"ON":"OFF"));
        recordDiagEvent("SWITCH", QString("主开关 %1").arg(on?"ON":"OFF"));
    });
    connect(btn_node_detection_,&SmoothToggle::toggled,[this](bool on){
        node_detection_done_ = on;
        log_output_->append(QString("[感知] %1").arg(on?"就绪":"未就绪"));
        recordDiagEvent("SWITCH", QString("感知就绪 %1").arg(on?"ON":"OFF"));
    });
    connect(btn_node_drive_,&QPushButton::clicked,[this]{ node_walk_mode_=2; log_output_->append(QString("[直行] X=%1m v=%2").arg(spin_x_terminal_->value()).arg(spin_v_ref_->value())); recordDiagEvent("BTN", QString("直行 X=%1 v=%2").arg(spin_x_terminal_->value()).arg(spin_v_ref_->value())); });
    connect(btn_node_rotate_,&QPushButton::clicked,[this]{ node_walk_mode_=1; log_output_->append(QString("[旋转] θ=%1° ω=%2").arg(spin_theta_terminal_->value()).arg(spin_omega_ref_->value())); recordDiagEvent("BTN", QString("旋转 θ=%1 ω=%2").arg(spin_theta_terminal_->value()).arg(spin_omega_ref_->value())); });
    connect(btn_node_stop_,&QPushButton::clicked,[this]{ node_walk_mode_=0; log_output_->append("[停止]"); recordDiagEvent("BTN", "停止"); });
    connect(chk_node_mold_enable_,&QCheckBox::toggled,[this](bool c){ log_output_->append(QString("[铲刀PID] %1").arg(c?"使能":"关闭")); });

    // Tab3
    connect(btn_send_walk_params_,&QPushButton::clicked,[this]{
        std_msgs::Float64MultiArray m; m.data.resize(17);
        m.data[0]=spin_x_tol_->value(); m.data[1]=0.1; m.data[2]=spin_theta_tol_->value();
        m.data[3]=1.0; m.data[4]=5.0; m.data[5]=1.0; m.data[6]=spin_track_width_->value();
        m.data[7]=spin_kp_x_->value(); m.data[8]=spin_ki_x_->value(); m.data[9]=spin_kd_x_->value();
        m.data[10]=spin_kp_theta_->value(); m.data[11]=spin_ki_theta_->value(); m.data[12]=spin_kd_theta_->value();
        m.data[13]=spin_td_r_->value(); m.data[14]=spin_td_h_->value();
        m.data[15]=spin_gear_dz_->value(); m.data[16]=spin_steer_dz_->value();
        pub_walk_params_.publish(m);
        label_walk_params_ack_->setText(QString("\u2713 Kp_x=%1 Ki_x=%2 | 挡位死区=%3 转向死区=%4")
            .arg(m.data[7],0,'f',2).arg(m.data[8],0,'f',3)
            .arg(m.data[15],0,'f',0).arg(m.data[16],0,'f',0));
        label_walk_params_ack_->setStyleSheet("color:#34C759;");
        LOG_INFO("Walk params sent (17 values)");
        SmoothUI::flashSuccess(btn_send_walk_params_, "✓ 已发送");
    });
    connect(btn_send_mold_params_,&QPushButton::clicked,[this]{
        std_msgs::Float64MultiArray m; m.data.resize(22);
        m.data[0]=spin_kp_h_up_->value(); m.data[1]=spin_kp_h_dn_->value();
        m.data[2]=spin_ki_h_up_->value(); m.data[3]=spin_ki_h_dn_->value();
        m.data[4]=200; m.data[5]=400; m.data[6]=800;
        m.data[7]=spin_dz_height_->value(); m.data[8]=100; m.data[9]=spin_imax_height_->value(); m.data[10]=10;
        m.data[11]=spin_kp_t_up_->value(); m.data[12]=spin_kp_t_dn_->value();
        m.data[13]=spin_ki_t_up_->value(); m.data[14]=spin_ki_t_dn_->value();
        m.data[15]=200; m.data[16]=400; m.data[17]=800;
        m.data[18]=spin_dz_theta_->value(); m.data[19]=100; m.data[20]=spin_imax_theta_->value(); m.data[21]=10;
        pub_mold_params_.publish(m);
        label_mold_params_ack_->setText(QString("✓ Kp_H=%1/%2 Ki_H=%3/%4 | Kp_θ=%5/%6 Ki_θ=%7/%8")
            .arg(m.data[0],0,'f',2).arg(m.data[1],0,'f',2).arg(m.data[2],0,'f',3).arg(m.data[3],0,'f',3)
            .arg(m.data[11],0,'f',2).arg(m.data[12],0,'f',2).arg(m.data[13],0,'f',3).arg(m.data[14],0,'f',3));
        label_mold_params_ack_->setStyleSheet("color:#34C759;");
        LOG_INFO("Mold params sent");
        SmoothUI::flashSuccess(btn_send_mold_params_, "✓ 已发送");
    });

    // Tab5
    connect(btn_send_plan_params_,&QPushButton::clicked,[this]{
        double angle = spin_shift_angle_->value();
        double heading = spin_push_heading_->value();
        // UI: index 0=底端斜移, index 1=顶端斜移, index 2=底端倒车斜移
        bool is_bot = (combo_path_mode_->selectedIndex() != 1);
        bool no_map = chk_test_mode_->isChecked();

        double push_len = no_map ? spin_push_length_->value() : 0.0;
        // 两种模式都使用面板上的列宽
        double blade_w = spin_blade_width_->value();
        int num_cols = no_map ? static_cast<int>(spin_test_columns_->value()) : 0;
        int ppc = no_map ? static_cast<int>(spin_test_passes_->value()) : 1;
        double map_scale = no_map ? 1.0 : spin_map_scale_->value();
        int start_corner = no_map ? 0 : combo_map_start_corner_->selectedIndex();
        double map_push_heading = no_map ? heading : spin_map_push_heading_->selectedIndex() * 90.0;

        // 1. 发送路径参数 (含辅助模块参数, 统一通过 /decision/path_params)
        // [0]push_length [1]blade_width [2]heading [3]v_push [4]v_reverse [5]omega_rotate
        // [6]x_back_set [7]pos_tol [8]theta_tol [9]num_columns [10]passes_per_col
        // [11]shift_angle [12]map_scale
        // [13]speed_gain_risk [14]speed_gain_mold [15]mold_limit
        // [16]overload_trans [17]overload_vehicle [18]overload_angular
        // [19]start_corner (0=左下 1=右下 2=左上 3=右上)
        // [20]map_push_heading (有地图推土方向, 度)
        // [21]level_mode (0=找平+横坡, 1=纵坡+横坡)
        // [22]target_level_height (米, ENU-Up, 找平+横坡用)
        // [23]blade_angle_deg (度, 横坡角度, 两个模式共用)
        // [24]slope_start_height (米, ENU-Up, 纵坡+横坡用)
        // [25]slope_gradient (%, 纵向坡度, 纵坡+横坡用)
        std_msgs::Float64MultiArray pp;
        pp.data = {push_len, blade_w, heading,
                   spin_v_push_->value(), spin_v_reverse_->value(), spin_omega_rotate_->value(),
                   spin_x_back_set_->value(), spin_path_pos_tol_->value(), spin_path_theta_tol_->value(),
                   static_cast<double>(num_cols), static_cast<double>(ppc), angle, map_scale,
                   spin_speed_gain_risk_->value(), spin_speed_gain_mold_->value(), spin_mold_limit_->value(),
                   spin_overload_trans_->value(), spin_overload_vehicle_->value(), spin_overload_angular_->value(),
                   static_cast<double>(start_corner), map_push_heading,
                   static_cast<double>(combo_level_mode_->selectedIndex()),
                   spin_target_level_height_->value(),
                   spin_blade_angle_deg_->value(),
                   spin_slope_start_height_->value(),
                   spin_slope_gradient_->value()};
        pub_path_params_.publish(pp);

        // 2. 测试模式标志
        {
            std_msgs::Float64MultiArray tc;
            tc.data.resize(6);
            tc.data[0] = no_map ? 1.0 : 0.0;
            tc.data[1] = push_len;
            tc.data[2] = blade_w * std::max(num_cols, 1);
            tc.data[3] = num_cols;
            tc.data[4] = 0;
            tc.data[5] = heading;
            pub_test_config_.publish(tc);
        }

        if (no_map) {
            QString mode_name = is_bot ? "\u5E95\u7AEF\u659C\u79FB" : "\u9876\u7AEF\u659C\u79FB";
            label_plan_params_ack_->setText(QString("\u2713 \u65E0\u5730\u56FE %1: %2m\u00D7%3\u5217\u00D7%4\u904D  \u89D2\u5EA6=%5\u00B0  \u5217\u5BBD=%6m")
                .arg(mode_name).arg(push_len,0,'f',1).arg(num_cols).arg(ppc)
                .arg(angle,0,'f',0).arg(blade_w,0,'f',1));
        } else {
            QString mode_name = is_bot ? "\u5E95\u7AEF\u659C\u79FB" : "\u9876\u7AEF\u659C\u79FB";
            label_plan_params_ack_->setText(QString("\u2713 \u6709\u5730\u56FE %1: \u89D2\u5EA6=%2\u00B0  \u7F29\u653E=%3  \u5217\u5BBD=%4m")
                .arg(mode_name).arg(angle,0,'f',0).arg(map_scale,0,'f',1).arg(blade_w,0,'f',1));
        }
        label_plan_params_ack_->setStyleSheet("color:#34C759;");
        SmoothUI::flashSuccess(btn_send_plan_params_, "✓ 已发送");
        // 规划参数是"重操作"(影响整个路径生成/执行),
        // 用 iOS 风 SuccessHUD 做全局强反馈 (融合风补充五)
        SuccessHUD::show(this, "参数已应用");
    });

    // 脚本执行/停止
    connect(btn_run_script_,&QPushButton::clicked,[this]{
        QString script = edit_test_script_->toPlainText().trimmed();
        if (script.isEmpty()) return;
        // 发给 decision 执行 + 生成路径 + 发布地图显示
        std_msgs::String msg;
        msg.data = script.toStdString();
        pub_test_script_.publish(msg);
        btn_run_script_->setEnabled(false); btn_stop_script_->setEnabled(true);
        label_script_status_->setText("\u6267\u884C\u4E2D...");
        label_script_status_->setStyleSheet("color:#FF9500;font-weight:bold;");
    });
    connect(btn_stop_script_,&QPushButton::clicked,[this]{
        std_msgs::String msg;
        msg.data = "STOP";
        pub_test_script_.publish(msg);
        btn_run_script_->setEnabled(true); btn_stop_script_->setEnabled(false);
        label_script_status_->setText("\u5DF2\u505C\u6B62");
        label_script_status_->setStyleSheet("color:#AEAEB2;");
    });

    // 生成路径按钮: 先发送参数, 再触发 decision 生成路径
    connect(btn_generate_path_,&QPushButton::clicked,[this]{
        // 1. 发送换列模式 (UI index → PathMode enum)
        // UI: 0=底端斜移(UNIDIRECTIONAL=1), 1=顶端斜移(ZIGZAG=0), 2=底端倒车斜移(UNIDI_BACK_SHIFT=2)
        std_msgs::Float64 pm;
        int ui_idx = combo_path_mode_->selectedIndex();
        pm.data = (ui_idx == 0) ? 1.0 : (ui_idx == 1) ? 0.0 : 2.0;
        pub_path_mode_.publish(pm);
        // 2. 发送其他规划参数
        btn_send_plan_params_->click();
        // 3. 短延迟后发送重新生成指令
        QTimer::singleShot(200, [this]{
            std_msgs::Float64 msg;
            msg.data = 1.0;
            pub_regen_path_.publish(msg);
            label_plan_params_ack_->setText(label_plan_params_ack_->text() + " → \u8DEF\u5F84\u751F\u6210\u4E2D...");
            label_plan_params_ack_->setStyleSheet("color:#007AFF;");
            // iOS HUD 全局反馈 (融合风补充五): 路径生成是重操作
            SuccessHUD::show(this, "路径已生成");
        });
        recordDiagEvent("BTN", QString("生成路径 mode=%1 push=%2m blade=%3m angle=%4")
            .arg(ui_idx).arg(spin_push_length_->value()).arg(spin_blade_width_->value()).arg(spin_shift_angle_->value()));
    });

    // 紧急停止: 车直接不动 + 清路径
    connect(btn_path_stop_,&QPushButton::clicked,[this]{
        // 1. 全车紧急停止 (刹车+CAN清零+停定时器)
        emergencyStop();
        // 2. 清除决策层路径
        std_msgs::Float64 msg;
        msg.data = 1.0;
        pub_path_stop_.publish(msg);
        btn_start_exec_->setEnabled(true);
        btn_pause_exec_->setEnabled(false);
        label_plan_params_ack_->setText("\u26A0 \u7D27\u6025\u505C\u6B62 — \u8F66\u8F86\u5DF2\u505C + \u8DEF\u5F84\u5DF2\u6E05");
        label_plan_params_ack_->setStyleSheet("color:#FF3B30;font-weight:bold;");
        recordDiagEvent("BTN", "紧急停止");
    });

    // 开始执行: 发送参数 + 生成路径 + 开启主开关和感知 → 自动跑
    connect(btn_start_exec_,&QPushButton::clicked,[this]{
        recordDiagEvent("BTN", "开始执行");
        // 1. 先生成路径 (会自动发送参数)
        btn_generate_path_->click();

        // 2. 延迟后开启执行
        QTimer::singleShot(500, [this]{
            // 切到自动作业模式
            current_mode_ = MODE_AUTO;
            mode_group_->setSelectedIndex(MODE_AUTO);

            // 开主开关 + 感知就绪
            node_main_switch_on_ = true;
            node_detection_done_ = true;
            btn_node_main_switch_->setChecked(true);   // 触发 toggled(true), 同步状态和 log
            btn_node_detection_->setChecked(true);

            // 确保节点定时器运行
            if (!node_timer_->isActive()) node_timer_->start(20);

            btn_start_exec_->setEnabled(false);
            btn_pause_exec_->setEnabled(true);
            label_plan_params_ack_->setText("\u25B6 \u6267\u884C\u4E2D...");
            label_plan_params_ack_->setStyleSheet("color:#34C759;font-weight:bold;");
        });
    });

    // 暂停执行: 关闭主开关 → REMOTE_TAKEOVER (停车, 保留路径)
    connect(btn_pause_exec_,&QPushButton::clicked,[this]{
        node_main_switch_on_ = false;
        btn_node_main_switch_->setChecked(false);
        btn_start_exec_->setEnabled(true);
        btn_pause_exec_->setEnabled(false);
        label_plan_params_ack_->setText("\u23F8 \u5DF2\u6682\u505C — \u70B9\u201C\u5F00\u59CB\u6267\u884C\u201D\u7EE7\u7EED");
        label_plan_params_ack_->setStyleSheet("color:#FF9500;");
        recordDiagEvent("BTN", "暂停执行");
    });

    // Tab6
    connect(btn_save_config_,&QPushButton::clicked,[this]{
        QString path = edit_config_path_->text().trimmed();
        if (path.isEmpty()) path = "./params/default.txt";
        QDir().mkpath(QFileInfo(path).absolutePath());
        edit_config_path_->setText(path);
        saveConfig(path);
    });
    connect(btn_load_config_,&QPushButton::clicked,[this]{
        QString path = QFileDialog::getOpenFileName(this,"\u52A0\u8F7D\u914D\u7F6E",edit_config_path_->text(),"Config (*.txt)");
        if(path.isEmpty()) return;
        edit_config_path_->setText(path);
        loadConfig(path);
    });
    connect(btn_load_and_send_,&QPushButton::clicked,[this]{
        QString path = QFileDialog::getOpenFileName(this,"\u52A0\u8F7D\u5E76\u53D1\u9001",edit_config_path_->text(),"Config (*.txt)");
        if(path.isEmpty()) return;
        edit_config_path_->setText(path);

        // \u9636\u6BB5\u56DB \u00B7 \u52A8\u6548\u6253\u78E8: \u5E94\u7528\u8FC7\u7A0B\u663E\u793A ActivityIndicator, \u5B8C\u6210\u540E\u5F39 SuccessHUD
        auto* spinner = new ActivityIndicator(btn_load_and_send_, 16);
        spinner->move((btn_load_and_send_->width()  - 16) / 2,
                      (btn_load_and_send_->height() - 16) / 2);
        spinner->start();

        loadConfig(path);
        btn_send_walk_params_->click();
        btn_send_mold_params_->click();
        btn_send_plan_params_->click();
        LOG_INFO("Config loaded and sent to all nodes");

        // 350ms \u540E\u505C\u8F6C, SuccessHUD \u7ED9\u6700\u7EC8\u53CD\u9988
        QPointer<ActivityIndicator> spinnerGuard(spinner);
        QPointer<ManualControlWidget> selfGuard(this);
        QTimer::singleShot(350, this, [spinnerGuard, selfGuard]() {
            if (spinnerGuard) {
                spinnerGuard->stop();
                spinnerGuard->deleteLater();
            }
            if (selfGuard) {
                // "\u914D\u7F6E\u5DF2\u5E94\u7528" = \u914D\u7F6E\u5DF2\u5E94\u7528
                SuccessHUD::show(selfGuard.data(),
                    QString::fromUtf8("\xe9\x85\x8d\xe7\xbd\xae\xe5\xb7\xb2\xe5\xba\x94\xe7\x94\xa8"));
            }
        });
    });
    // 预设管理
    connect(btn_save_preset_,&QPushButton::clicked,[this]{
        bool ok;
        QString name = QInputDialog::getText(this, "\u53E6\u5B58\u4E3A\u9884\u8BBE", "\u9884\u8BBE\u540D\u79F0:", QLineEdit::Normal, "", &ok);
        if (!ok || name.trimmed().isEmpty()) return;
        QString dir = getParamsDir() + "/presets";
        QDir().mkpath(dir);
        QString path = dir + "/" + name.trimmed() + ".txt";
        saveConfig(path);
        refreshPresetList();
        // 选中刚保存的
        int idx = combo_vehicle_preset_->findText(name.trimmed());
        if (idx >= 0) combo_vehicle_preset_->setCurrentIndex(idx);
    });
    connect(btn_delete_preset_,&QPushButton::clicked,[this]{
        QString name = combo_vehicle_preset_->currentText();
        if (name.isEmpty() || name.startsWith("(")) return;
        QString path = getParamsDir() + "/presets/" + name + ".txt";
        if (QFile::exists(path)) {
            QFile::remove(path);
            refreshPresetList();
            LOG_INFO("Preset deleted: %s", name.toStdString().c_str());
        }
    });
    connect(combo_vehicle_preset_, QOverload<int>::of(&QComboBox::currentIndexChanged),[this](int){
        QString name = combo_vehicle_preset_->currentText();
        if (name.isEmpty() || name.startsWith("(")) return;
        QString path = getParamsDir() + "/presets/" + name + ".txt";
        if (QFile::exists(path)) {
            loadConfig(path);
            LOG_INFO("Preset loaded: %s", name.toStdString().c_str());
        }
    });
    // 历史记录
    connect(btn_restore_history_,&QPushButton::clicked,[this]{
        QString name = combo_history_->currentText();
        if (name.isEmpty() || name.startsWith("(")) return;
        QString path = getParamsDir() + "/history/" + name + ".txt";
        if (QFile::exists(path)) {
            loadConfig(path);
            LOG_INFO("History restored: %s", name.toStdString().c_str());
        }
    });
    connect(btn_view_changelog_,&QPushButton::clicked,[this]{
        QString logPath = getParamsDir() + "/changelog.log";
        QFile f(logPath);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::information(this, "\u53D8\u66F4\u65E5\u5FD7", "\u6682\u65E0\u53D8\u66F4\u8BB0\u5F55");
            return;
        }
        QString content = QTextStream(&f).readAll();
        f.close();
        // 只显示最后 5000 字符
        if (content.size() > 5000) content = "...\n" + content.right(5000);
        auto* dlg = new QDialog(this);
        dlg->setWindowTitle("\u53C2\u6570\u53D8\u66F4\u65E5\u5FD7");
        dlg->resize(600, 400);
        auto* vl = new QVBoxLayout(dlg);
        auto* te = new QTextEdit(dlg);
        te->setReadOnly(true);
        te->setFont(QFont("Monospace", 10));
        te->setPlainText(content);
        // 滚到底部
        te->moveCursor(QTextCursor::End);
        vl->addWidget(te);
        auto* closeBtn = new QPushButton("\u5173\u95ED", dlg);
        connect(closeBtn, &QPushButton::clicked, dlg, &QDialog::close);
        vl->addWidget(closeBtn);
        dlg->show();
    });

    // Tab7
    connect(btn_start_record_,&QPushButton::clicked,[this]{
        if(recording_) return;
        QString dir = edit_record_path_->text().trimmed();
        if (dir.isEmpty()) dir = QDir::homePath() + "/rosbag_records";
        QDir().mkpath(dir);
        QString filename = dir + "/record_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".bag";
        QStringList args; args << "record" << "-O" << filename;
        // 收集勾选的话题
        QStringList checkedTopics;
        for (int i = 0; i < record_topic_list_->count(); ++i) {
            auto* item = record_topic_list_->item(i);
            if (item->checkState() == Qt::Checked) checkedTopics << item->text();
        }
        if (!checkedTopics.isEmpty()) { for (const auto& t : checkedTopics) args << t; }
        else args << "-a";  // 没勾选任何话题则录制全部
        rosbag_process_ = new QProcess(this);
        rosbag_process_->start("rosbag", args);
        recording_ = true; record_elapsed_.start();
        btn_start_record_->setEnabled(false); btn_stop_record_->setEnabled(true);
        label_record_status_->setText(QString("\u5F55\u5236\u4E2D... (%1\u4E2A\u8BDD\u9898)").arg(checkedTopics.isEmpty() ? -1 : checkedTopics.size()));
        label_record_status_->setStyleSheet("color:#FF3B30;font-weight:bold;");
        LOG_INFO("Recording started: %s (%d topics)", filename.toStdString().c_str(), checkedTopics.size());
    });
    connect(btn_stop_record_,&QPushButton::clicked,[this]{
        if(!recording_||!rosbag_process_) return;
        rosbag_process_->terminate(); rosbag_process_->waitForFinished(3000);
        recording_ = false;
        btn_start_record_->setEnabled(true); btn_stop_record_->setEnabled(false);
        int sec = static_cast<int>(record_elapsed_.elapsed()/1000);
        label_record_status_->setText(QString("已停止 (录制%1秒)").arg(sec));
        label_record_status_->setStyleSheet("color:#34C759;");
        LOG_INFO("Recording stopped after %d seconds", sec);
    });
}

//==============================================================================
// 全局急停
//==============================================================================
void ManualControlWidget::emergencyStop() {
    if (!emergency_stopped_) {
        // === 全部停止 ===
        emergency_stopped_ = true;
        direct_timer_->stop(); node_timer_->stop();
        locked_ = true; unlock_pressed_ = false;
        forward_pressed_=backward_pressed_=left_pressed_=right_pressed_=false;
        gear_up_pressed_=gear_down_pressed_=false;
        blade_up_pressed_=blade_down_pressed_=blade_tilt_l_pressed_=blade_tilt_r_pressed_=false;
        std_msgs::Int16MultiArray l1; l1.data.resize(7,0); pub_lever1_.publish(l1);
        std_msgs::Int16MultiArray ml; ml.data.resize(4,0); pub_lever_moldboard_.publish(ml);
        std_msgs::Int32 br; br.data=1; pub_brake_.publish(br);
        brake_pressed_ = false;
        std_msgs::Float64 f; f.data=0;
        pub_main_switch_.publish(f); pub_main_switch_ctrl_.publish(f); pub_walk_state_.publish(f);
        node_main_switch_on_=false; node_walk_mode_=0;
        btn_emergency_stop_->setText("\u25B6 \u6062\u590D\u64CD\u4F5C");
        btn_emergency_stop_->setVariant(IOSButton::Success);   // 红 → 绿
        btn_node_main_switch_->setChecked(false);
        LOG_WARN("!!! ALL STOP !!!");
    } else {
        // === 恢复操作 ===
        emergency_stopped_ = false;
        btn_emergency_stop_->setText("\u26A0 \u5168\u90E8\u505C\u6B62");
        btn_emergency_stop_->setVariant(IOSButton::Danger);    // 绿 → 红
        // 根据当前Tab恢复对应定时器
        onTabChanged(tabs_->currentIndex());
        LOG_INFO("Operation resumed");
    }
}

//==============================================================================
// 底部抽屉: 折叠/展开
//==============================================================================
void ManualControlWidget::toggleDrawer() {
    int tab_bar_h = tabs_->tabBar()->sizeHint().height() + 4;
    auto* collapseBtn = tabs_->cornerWidget(Qt::TopRightCorner);

    drawer_anim_->stop();

    if (drawer_open_) {
        // 收起
        drawer_open_ = false;
        drawer_handle_->setVisible(false);
        drawer_anim_->setStartValue(tabs_->height());
        drawer_anim_->setEndValue(tab_bar_h);
        if (collapseBtn) qobject_cast<QLabel*>(collapseBtn)->setText("\u25B2");
        LOG_INFO("Drawer collapsed");
    } else {
        // 展开
        drawer_open_ = true;
        drawer_handle_->setVisible(true);
        tabs_->setMaximumHeight(16777215);
        drawer_anim_->setStartValue(tabs_->height());
        drawer_anim_->setEndValue(drawer_height_);
        if (collapseBtn) qobject_cast<QLabel*>(collapseBtn)->setText("\u25BC");
        LOG_INFO("Drawer expanded to %dpx", drawer_height_);
    }
    drawer_anim_->start();

    QMetaObject::Connection* conn = new QMetaObject::Connection();
    *conn = connect(drawer_anim_, &QPropertyAnimation::finished, this, [this, tab_bar_h, conn]() {
        if (!drawer_open_) {
            tabs_->setFixedHeight(tab_bar_h);
        } else {
            tabs_->setMinimumHeight(tab_bar_h + 50);
            tabs_->setMaximumHeight(this->height() - 200);
        }
        disconnect(*conn);
        delete conn;
    });
}

void ManualControlWidget::onTabBarClicked(int index) {
    if (!drawer_open_) {
        // 收起状态: 点任意 Tab → 展开
        tabs_->setCurrentIndex(index);
        toggleDrawer();
    } else if (index == tabs_->currentIndex()) {
        // 展开状态: 点当前 Tab → 收起
        toggleDrawer();
    }
    // 展开状态点其他 Tab → QTabWidget 自动切换
}

//==============================================================================
// 安全: Tab切换
//==============================================================================
void ManualControlWidget::onTabChanged(int index) {
    if (index == 0) {
        // Tab1 直接CAN — 启动直接控制, 停止节点控制
        direct_timer_->start(20);
        node_timer_->stop();
        node_walk_mode_ = 0;
        LOG_INFO("Switched to Tab1 (DirectCAN) — node timer stopped");
    } else if (index == 1 || index == 4) {
        // Tab2 节点测试 / Tab5 规划参数 — 需要节点定时器发布指令
        node_timer_->start(20);
        direct_timer_->stop();
        locked_ = true;
        unlock_pressed_ = false;
        forward_pressed_ = backward_pressed_ = left_pressed_ = right_pressed_ = false;
        if (index == 1) {
            btn_stop_->setStyleSheet("QPushButton{background:#D32F2F;color:white;border-radius:6px;font-weight:bold;}");
            btn_unlock_->setStyleSheet(STY_GRAY);
        }
        LOG_INFO("Switched to Tab%d — node timer active", index+1);
    } else {
        // 其他Tab — 两个控制定时器都停
        direct_timer_->stop();
        node_timer_->stop();
        locked_ = true;
        node_walk_mode_ = 0;
    }
}

//==============================================================================
// 定时回调
//==============================================================================
void ManualControlWidget::onDirectTimerTick() {
    publishLever1();
    publishMoldboard();
}

void ManualControlWidget::onNodeTimerTick() {
    publishNodeCommands();
    label_v_right_->setText(QString("V_right: %1").arg(v_right_val_,0,'f',1));
    label_v_left_->setText(QString("V_left: %1").arg(v_left_val_,0,'f',1));
    label_terminal_flag_->setText(QString("Terminal: %1").arg(terminal_flag_val_,0,'f',0));
    label_mold_debug_->setText(QString("铲刀 e_H:%1 e_θ:%2 u_H:%3 u_θ:%4").arg(mold_e_height_,0,'f',3).arg(mold_e_theta_,0,'f',1).arg(mold_u_height_,0,'f',0).arg(mold_u_theta_,0,'f',0));
}

void ManualControlWidget::onStatusTimerTick() {
    ros::spinOnce();  // 始终处理ROS回调, 不管当前在哪个Tab

    // 侧边栏更新 — 圆点+值同色, 状态一眼可见
    // 圆点用 SideDot::animateTo 做 300ms 颜色过渡; 值 label 仍瞬时切色 (刷新频率高动画反而抖动).
    auto setVal = [](QLabel* lbl, const QString& text, const QString& color) {
        lbl->setText(text);
        lbl->setStyleSheet(QString("color:%1;background:transparent;padding:0;"
                                   "font-weight:bold;font-family:Monospace;font-size:12px;").arg(color));
        QObject* dotObj = lbl->property("sidebar_dot").value<QObject*>();
        if (auto* sd = qobject_cast<SideDot*>(dotObj)) {
            sd->animateTo(QColor(color));
        }
    };

    setVal(side_main_switch_, node_main_switch_on_ ? "\u25CF ON" : "\u25CB OFF",
           node_main_switch_on_ ? "#34C759" : "#FF3B30");
    
    const char* exec_names[] = {"\u7A7A\u95F2","\u65CB\u8F6C\u4E2D","\u884C\u9A76\u4E2D","\u5230\u4F4D","\u5B8C\u6210","\u6025\u505C","\u8FC7\u8F7D\u540E\u9000"};
    int exec_idx = static_cast<int>(decision_status_val_);
    QString exec_color = (exec_idx==2||exec_idx==1) ? "#007AFF" : (exec_idx>=5) ? "#FF3B30" : "#AEAEB2";
    setVal(side_state_machine_, (exec_idx>=0&&exec_idx<=6) ? exec_names[exec_idx] : "--", exec_color);

    const char* walk_names[] = {"\u505C\u6B62","\u65CB\u8F6C","\u76F4\u884C","\u4E2D\u95F4\u70B9","\u7EC8\u70B9","\u51C6\u5907\u9000","\u63D0\u5200"};
    int ws = static_cast<int>(walk_state_val_);
    setVal(side_walk_state_, (ws>=0&&ws<=6) ? walk_names[ws] : QString::number(ws),
           (ws==2) ? "#34C759" : (ws==1) ? "#007AFF" : "#AEAEB2");

    setVal(side_heading_, QString("%1\u00B0").arg(heading_deg_,0,'f',1), "#E6F0FF");
    setVal(side_vehicle_speed_, QString("%1 m/s").arg(vehicle_speed_,0,'f',2),
           (std::abs(vehicle_speed_) > 0.1) ? "#34C759" : "#AEAEB2");
    setVal(side_engine_rpm_, QString("%1 rpm").arg(engine_rpm_,0,'f',0),
           (engine_rpm_ > 100) ? "#34C759" : "#FF9500");
    setVal(side_blade_height_, QString("R:%1 L:%2").arg(mold_right_mm_).arg(mold_left_mm_), "#007AFF");
    
    bool rtk_ok = (lla_lat_ != 0 && lla_lon_ != 0);
    setVal(side_rtk_status_, rtk_ok ? QString("%1, %2").arg(lla_lat_,0,'f',6).arg(lla_lon_,0,'f',6) : "\u65E0\u4FE1\u53F7",
           rtk_ok ? "#34C759" : "#FF3B30");
    
    bool can_ok = false;
    if (topic_diags_.contains("/LLA")) {
        qint64 now = diag_elapsed_.elapsed();
        can_ok = (now - topic_diags_["/LLA"].last_time_ms < 3000) && topic_diags_["/LLA"].last_time_ms > 0;
    }
    setVal(side_can_status_, can_ok ? "\u25CF \u6B63\u5E38" : "\u25CB \u65AD\u5F00", can_ok ? "#34C759" : "#FF3B30");

    // 侧边栏 — 感知风险状态灯
    {
        qint64 now_ms = diag_elapsed_.elapsed();
        bool risk_timeout = (risk_state_last_ms_ == 0) || (now_ms - risk_state_last_ms_ > 3000);
        QLabel* risk_lights[] = {side_risk_front_, side_risk_back_, side_risk_left_, side_risk_right_};
        const char* risk_labels[] = {"\u524D", "\u540E", "\u5DE6", "\u53F3"};
        for (int i = 0; i < 4; ++i) {
            QString bg, text;
            if (risk_timeout) {
                bg = "#AEAEB2"; text = risk_labels[i];   // 灰色: 无数据/超时
            } else if (risk_state_[i] == 0) {
                bg = "#34C759"; text = QString("%1 \u5B89\u5168").arg(risk_labels[i]); // 绿色
            } else if (risk_state_[i] == 1) {
                bg = "#FF9500"; text = QString("%1 \u5371\u9669").arg(risk_labels[i]); // 黄色
            } else if (risk_state_[i] == 2) {
                bg = "#FF3B30"; text = QString("%1 \u505C\u8F66").arg(risk_labels[i]); // 红色
            } else {
                bg = "#AEAEB2"; text = risk_labels[i];
            }
            risk_lights[i]->setText(text);
            risk_lights[i]->setStyleSheet(
                QString("background:%1;color:white;border-radius:6px;font-weight:bold;font-size:9px;").arg(bg));
        }
    }

    // Tab4 — 整车状态刷新 (仪表盘风格)
    // 核心圆形仪表
    gauge_engine_rpm_->setValue(engine_rpm_);
    gauge_vehicle_speed_->setValue(vehicle_speed_);
    gauge_coolant_temp_->setValue(coolant_temp_);
    gauge_fuel_level_->setValue(fuel_level_);

    // 姿态水平仪 (Roll 用铲刀 IMU 数据, 参见 Fix-v19)
    bar_pitch_->setValue(imu_pitch_deg_);
    bar_roll_->setValue(mold_imu_roll_deg_);

    // 状态胶囊
    pill_hand_brake_->setState(hand_brake_ ? StatusPill::PILL_DANGER : StatusPill::PILL_ON,
                               hand_brake_ ? "\u62C9\u8D77" : "\u91CA\u653E");
    pill_hydraulic_lock_->setState(hydraulic_lock_ ? StatusPill::PILL_WARN : StatusPill::PILL_ON,
                                   hydraulic_lock_ ? "\u9501\u5B9A" : "\u89E3\u9501");
    pill_brake_valve_->setState(brake_valve_ ? StatusPill::PILL_ON : StatusPill::PILL_OFF,
                                brake_valve_ ? "\u5F00" : "\u5173");
    pill_manual_auto_->setState(manual_auto_ ? StatusPill::PILL_NORMAL : StatusPill::PILL_WARN,
                                manual_auto_ ? "\u81EA\u52A8" : "\u624B\u52A8");

    // Tab1 挡位方向显示 (N/F/R)
    const char* gear_dirs[] = {"\u7A7A\u6321", "\u524D\u8FDB", "\u540E\u9000"};
    QString dir_text = (gear_dir_ == 0) ? "N" : (gear_dir_ == 1) ? "F" : (gear_dir_ == 2) ? "R" : "-";
    QString dir_color = (gear_dir_ == 0) ? "#AEAEB2" : (gear_dir_ == 1) ? "#34C759" : "#FF9500";
    label_gear_dir_display_->setText(dir_text);
    label_gear_dir_display_->setStyleSheet(QString("background:white;color:%1;border-radius:10px;border:2px solid %1;font-size:18px;font-weight:bold;").arg(dir_color));
    QString pos_text = (gear_pos_ > 0) ? QString::number(gear_pos_) : "-";
    label_gear_display_->setText(pos_text);
    label_gear_display_->setStyleSheet(QString("background:white;color:%1;border-radius:10px;border:2px solid %1;font-size:20px;font-weight:bold;").arg(dir_color));

    // Tab4 辅助文字指标
    label_engine_torque_->setText(QString("%1 %%").arg(engine_torque_, 0, 'f', 0));
    label_engine_hours_->setText(QString("%1 h").arg(engine_hours_, 0, 'f', 1));
    label_trans_speed_->setText(QString("%1 rpm").arg(trans_speed_, 0, 'f', 0));
    label_gear_pos_->setText(QString("%1 \u6321").arg(gear_pos_));
    label_gear_dir_->setText((gear_dir_ <= 2) ? gear_dirs[gear_dir_] : "--");
    label_fuel_total_->setText(QString("%1 L").arg(fuel_total_, 0, 'f', 1));
    label_handle_turn_->setText(QString("%1").arg(handle_turn_val_));
    label_output_curr_->setText(QString("[%1, %2, %3, %4]").arg(output_curr_[0]).arg(output_curr_[1]).arg(output_curr_[2]).arg(output_curr_[3]));
    label_lla_detail_->setText(QString("%1, %2, %3m").arg(lla_lat_, 0, 'f', 7).arg(lla_lon_, 0, 'f', 7).arg(lla_alt_, 0, 'f', 2));
    label_heading_detail_->setText(QString("%1\u00B0").arg(heading_deg_, 0, 'f', 1));

    // 故障汇总 (置顶横幅)
    fault_count_ = 0;
    QString faults;
    if (mil_light_) { faults += "\u53D1\u52A8\u673A\u6545\u969C  "; fault_count_++; }
    if (red_stop_) { faults += "\u505C\u8F66\u62A5\u8B66  "; fault_count_++; }
    if (sys_error_code_ != 0) { faults += QString("VCU:0x%1  ").arg(sys_error_code_, 0, 16); fault_count_++; }
    if (coolant_temp_ > 105) { faults += "\u6C34\u6E29\u8FC7\u9AD8  "; fault_count_++; }
    if (fuel_level_ < 10) { faults += "\u71C3\u6CB9\u4E0D\u8DB3  "; fault_count_++; }
    if (fault_count_ == 0) {
        label_fault_summary_->setText("\u2705  \u65E0\u6545\u969C");
        label_fault_summary_->setStyleSheet(
            "QLabel{background:#E8F9EC;color:#2FA350;border:1px solid #A1E0AF;"
            "border-radius:10px;font-size:13px;font-weight:bold;padding:8px;}");
    } else {
        label_fault_summary_->setText(QString("\u26A0  %1\u9879\u6545\u969C: %2").arg(fault_count_).arg(faults.trimmed()));
        label_fault_summary_->setStyleSheet(
            "QLabel{background:#FFEEED;color:#C02825;border:1px solid #FFB3AF;"
            "border-radius:10px;font-size:13px;font-weight:bold;padding:8px;}");
    }

    // Tab5 地图信息
    if (map_rows_ > 0 && map_cols_ > 0) {
        label_map_info_->setText(QString("\u5730\u56FE: %1\u00D7%2 \u683C, \u5206\u8FA8\u7387 %3m \u2192 %4\u00D7%5 m")
            .arg(map_rows_).arg(map_cols_).arg(map_res_,0,'f',2)
            .arg(map_rows_*map_res_,0,'f',1).arg(map_cols_*map_res_,0,'f',1));
        label_map_info_->setStyleSheet("color:#34C759;padding:4px;");
    }

    // Tab8 通信诊断
    qint64 now = diag_elapsed_.elapsed();
    for (auto it = topic_diags_.begin(); it != topic_diags_.end(); ++it) {
        auto& d = it.value();
        double dt = (now - d.last_time_ms) / 1000.0;
        if (d.last_time_ms == 0) {
            d.freq_label->setText("-- Hz");
            d.status_label->setText("无数据"); d.status_label->setStyleSheet("color:#AEAEB2;");
            if (d.value_label) { d.value_label->setText("--"); d.value_label->setStyleSheet("color:#AEAEB2;"); }
        } else if (dt > 3.0) {
            d.freq_label->setText("0 Hz");
            d.status_label->setText("超时"); d.status_label->setStyleSheet("color:#FF3B30;font-weight:bold;");
            if (d.value_label) d.value_label->setStyleSheet("color:#FF3B30;");
        } else {
            d.freq_label->setText(QString("%1 Hz").arg(d.count / std::max(dt, 0.2), 0, 'f', 0));
            d.status_label->setText("正常"); d.status_label->setStyleSheet("color:#34C759;");
            if (d.value_label) {
                d.value_label->setText(d.last_value.isEmpty() ? "--" : d.last_value);
                d.value_label->setStyleSheet("color:#000000;");
            }
        }
        d.count = 0;
        d.last_time_ms = now;
    }

    // Tab8 硬件在线状态: 看对应话题有没有数据
    auto hwStatus = [&](StatusPill* pill, const QString& topic) {
        bool online = false;
        if (topic_diags_.contains(topic)) {
            auto& d = topic_diags_[topic];
            double dt = (now - d.last_time_ms) / 1000.0;
            online = (d.last_time_ms > 0 && dt < 3.0);
        }
        pill->setState(online ? StatusPill::PILL_ON : StatusPill::PILL_DANGER,
                       online ? "\u5728\u7EBF" : "\u79BB\u7EBF");
    };
    hwStatus(label_hw_rtk_, "/LLA");
    hwStatus(label_hw_imu_, "/AHRS_IMU");
    hwStatus(label_hw_engine_, "/Engine_Speed_Actual");
    hwStatus(label_hw_can_, "/U_Lever1");

    // 通信超时保护 (无地图模式跳过, 不依赖感知)
    if (!emergency_stopped_ && !chk_test_mode_->isChecked()
        && (direct_timer_->isActive() || node_timer_->isActive())) {
        qint64 now_ms = diag_elapsed_.elapsed();
        if (topic_diags_.contains("/LLA") && topic_diags_["/LLA"].last_time_ms > 0) {
            if ((now_ms - topic_diags_["/LLA"].last_time_ms) / 1000.0 > 3.0) {
                LOG_WARN("COMM TIMEOUT → EMERGENCY STOP"); emergencyStop(); } } }

    // 录制时间更新
    if (recording_) {
        int sec = static_cast<int>(record_elapsed_.elapsed()/1000);
        label_record_status_->setText(QString("录制中... %1秒").arg(sec));
    }
}

//==============================================================================
// 指令发布
//==============================================================================
void ManualControlWidget::publishLever1() {
    std_msgs::Int16MultiArray m; m.data.resize(7,0);
    if(locked_){ pub_lever1_.publish(m); return; }
    int sv=slider_steering_->value();
    if(left_pressed_){m.data[0]=1;m.data[2]=sv;} if(right_pressed_){m.data[1]=1;m.data[2]=sv;}
    if(forward_pressed_) m.data[5]=1; else if(backward_pressed_) m.data[5]=2;
    m.data[3]=gear_up_pressed_?1:0; m.data[4]=gear_down_pressed_?1:0; m.data[6]=unlock_pressed_?1:0;
    pub_lever1_.publish(m);

    // 无人/有人模式: 持续发送 /Mode_Switch → ros_to_can → CAN
    std_msgs::Int32 mode_sw;
    mode_sw.data = btn_unmanned_mode_->isChecked() ? 1 : 0;
    pub_mode_switch_.publish(mode_sw);

    // 铲刀使能: 主开关关闭时由 GUI 控制, 主开关开启时由 decision 接管
    if (!node_main_switch_on_) {
        std_msgs::Int16 mold_flag;
        mold_flag.data = chk_blade_enable_->isChecked() ? 1 : 0;
        pub_moldboard_flag_direct_.publish(mold_flag);
    }

    // 刹车: 按住发1, 松开发0 (50Hz持续)
    std_msgs::Int32 brk;
    brk.data = brake_pressed_ ? 1 : 0;
    pub_brake_.publish(brk);
}
void ManualControlWidget::publishMoldboard() {
    std_msgs::Int16MultiArray m; m.data.resize(4,0);
    if(locked_){pub_lever_moldboard_.publish(m);return;}
    int16_t s=static_cast<int16_t>(slider_blade_speed_->value());
    // D0-D1=斜拉缩(左升右降), D2-D3=斜拉伸(左降右升), D4-D5=铲刀升, D6-D7=铲刀降
    if(blade_tilt_l_pressed_)m.data[0]=s; if(blade_tilt_r_pressed_)m.data[1]=s;
    if(blade_up_pressed_)m.data[2]=s; if(blade_down_pressed_)m.data[3]=s;
    pub_lever_moldboard_.publish(m);
}
void ManualControlWidget::publishNodeCommands() {
    std_msgs::Float64 f;
    switch(current_mode_) {
    case MODE_AUTO:
        f.data=node_main_switch_on_?1:0; pub_main_switch_.publish(f); pub_main_switch_ctrl_.publish(f);
        f.data=node_detection_done_?1:0; pub_detection_.publish(f);
        break;
    case MODE_WALK: {
        f.data=1; pub_main_switch_ctrl_.publish(f);
        f.data=node_walk_mode_; pub_walk_state_.publish(f);
        std_msgs::Float64MultiArray t; t.data.resize(3,0);
        if(node_walk_mode_==2) t.data[0]=spin_x_terminal_->value();
        else if(node_walk_mode_==1) t.data[2]=spin_theta_terminal_->value();
        pub_terminal_.publish(t);
        std_msgs::Float64MultiArray r; r.data.resize(2,0);
        if(node_walk_mode_==2) r.data[0]=spin_v_ref_->value();
        else if(node_walk_mode_==1) r.data[1]=spin_omega_ref_->value();
        pub_reference_.publish(r);
        break; }
    case MODE_BLADE:
        f.data=1; pub_main_switch_ctrl_.publish(f);
        f.data=chk_node_mold_enable_->isChecked()?1:0; pub_mold_ctrl_flag_.publish(f);
        f.data=spin_ref_height_->value(); pub_ref_height_.publish(f);
        f.data=spin_ref_angle_->value(); pub_ref_angle_.publish(f);
        break;
    }
}

//==============================================================================
// 持久化: 保存配置到txt
//==============================================================================
QString ManualControlWidget::getParamsDir() const {
    QFileInfo fi(edit_config_path_->text());
    return fi.absolutePath();
}

void ManualControlWidget::refreshPresetList() {
    combo_vehicle_preset_->clear();
    QString dir = getParamsDir() + "/presets";
    QDir d(dir);
    if (d.exists()) {
        auto files = d.entryList({"*.txt"}, QDir::Files, QDir::Name);
        for (const auto& f : files)
            combo_vehicle_preset_->addItem(QFileInfo(f).baseName());
    }
    if (combo_vehicle_preset_->count() == 0)
        combo_vehicle_preset_->addItem("(\u65E0\u9884\u8BBE)");
}

void ManualControlWidget::refreshHistoryList() {
    combo_history_->clear();
    QString dir = getParamsDir() + "/history";
    QDir d(dir);
    if (d.exists()) {
        auto files = d.entryList({"*.txt"}, QDir::Files, QDir::Time);  // 按时间排序, 最新在前
        for (const auto& f : files)
            combo_history_->addItem(QFileInfo(f).baseName());
    }
    if (combo_history_->count() == 0)
        combo_history_->addItem("(\u65E0\u5386\u53F2)");
}

void ManualControlWidget::saveHistory(const QString& configPath) {
    QString baseDir = getParamsDir();

    // 1. 复制到 history/ 目录
    QString histDir = baseDir + "/history";
    QDir().mkpath(histDir);
    QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    QString histFile = histDir + "/" + ts + ".txt";
    QFile::copy(configPath, histFile);
    LOG_INFO("History saved: %s", histFile.toStdString().c_str());

    // 2. 生成变更日志 (对比当前值和上次保存的值)
    // 收集当前参数
    QMap<QString, double> currentParams;
    currentParams["Kp_x"] = spin_kp_x_->value();
    currentParams["Ki_x"] = spin_ki_x_->value();
    currentParams["Kd_x"] = spin_kd_x_->value();
    currentParams["Kp_theta"] = spin_kp_theta_->value();
    currentParams["Ki_theta"] = spin_ki_theta_->value();
    currentParams["Kd_theta"] = spin_kd_theta_->value();
    currentParams["X_Tolerance"] = spin_x_tol_->value();
    currentParams["Theta_Tolerance"] = spin_theta_tol_->value();
    currentParams["track_width"] = spin_track_width_->value();
    currentParams["TD_r"] = spin_td_r_->value();
    currentParams["TD_h"] = spin_td_h_->value();
    currentParams["Gear_Deadzone"] = spin_gear_dz_->value();
    currentParams["Steer_Deadzone"] = spin_steer_dz_->value();
    currentParams["Kp_Height_Up"] = spin_kp_h_up_->value();
    currentParams["Kp_Height_Down"] = spin_kp_h_dn_->value();
    currentParams["Ki_Height_Up"] = spin_ki_h_up_->value();
    currentParams["Ki_Height_Down"] = spin_ki_h_dn_->value();
    currentParams["Deadzone_Height"] = spin_dz_height_->value();
    currentParams["I_MAX_Height"] = spin_imax_height_->value();
    currentParams["Kp_Theta_Up"] = spin_kp_t_up_->value();
    currentParams["Kp_Theta_Down"] = spin_kp_t_dn_->value();
    currentParams["Ki_Theta_Up"] = spin_ki_t_up_->value();
    currentParams["Ki_Theta_Down"] = spin_ki_t_dn_->value();
    currentParams["Deadzone_Theta"] = spin_dz_theta_->value();
    currentParams["I_MAX_Theta"] = spin_imax_theta_->value();
    currentParams["Push_Length"] = spin_push_length_->value();
    currentParams["Blade_Width"] = spin_blade_width_->value();
    currentParams["Shift_Angle"] = spin_shift_angle_->value();
    currentParams["Push_Heading"] = spin_push_heading_->value();
    currentParams["V_Push"] = spin_v_push_->value();
    currentParams["V_Reverse"] = spin_v_reverse_->value();
    currentParams["Omega_Rotate"] = spin_omega_rotate_->value();
    currentParams["X_Back_Set"] = spin_x_back_set_->value();
    currentParams["Pos_Tolerance"] = spin_path_pos_tol_->value();
    currentParams["Speed_Gain_Risk"] = spin_speed_gain_risk_->value();
    currentParams["Speed_Gain_Mold"] = spin_speed_gain_mold_->value();
    currentParams["Mold_Limit"] = spin_mold_limit_->value();
    currentParams["Overload_Trans"] = spin_overload_trans_->value();
    currentParams["Overload_Vehicle"] = spin_overload_vehicle_->value();
    currentParams["Overload_Angular"] = spin_overload_angular_->value();

    // 写变更日志
    QString logFile = baseDir + "/changelog.log";
    QFile log(logFile);
    if (log.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream ls(&log);
        int changes = 0;
        ls << "\n[" << ts << "] \u4FDD\u5B58\u914D\u7F6E: " << configPath << "\n";
        for (auto it = currentParams.begin(); it != currentParams.end(); ++it) {
            if (last_saved_params_.contains(it.key())) {
                double old_val = last_saved_params_[it.key()];
                if (std::abs(old_val - it.value()) > 1e-6) {
                    ls << "  " << it.key() << ": " << old_val << " \u2192 " << it.value() << "\n";
                    changes++;
                }
            }
        }
        if (last_saved_params_.isEmpty()) {
            ls << "  (\u9996\u6B21\u4FDD\u5B58, \u65E0\u5BF9\u6BD4\u57FA\u51C6)\n";
        } else if (changes == 0) {
            ls << "  (\u65E0\u53C2\u6570\u53D8\u66F4)\n";
        }
        log.close();
        LOG_INFO("Changelog updated: %d changes", changes);
    }

    // 更新上次保存的参数快照
    last_saved_params_ = currentParams;

    // 刷新历史列表
    refreshHistoryList();
}

void ManualControlWidget::saveConfig(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        LOG_ERROR("Cannot open file for writing: %s", path.toStdString().c_str());
        QMessageBox::critical(this, "\u4FDD\u5B58\u5931\u8D25",
            QString("\u65E0\u6CD5\u5199\u5165\u6587\u4EF6:\n%1\n\n\u8BF7\u68C0\u67E5\u8DEF\u5F84\u548C\u6743\u9650").arg(path));
        return;
    }
    QTextStream out(&file);
    out.setRealNumberPrecision(6);

    out << "# 推土机控制参数配置\n";
    out << "# 生成时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") << "\n";
    out << "# 车辆型号: " << combo_vehicle_preset_->currentText() << "\n\n";

    out << "# ===== 行走 PID (Tab3) =====\n";
    out << "Kp_x=" << spin_kp_x_->value() << "\n";
    out << "Ki_x=" << spin_ki_x_->value() << "\n";
    out << "Kd_x=" << spin_kd_x_->value() << "\n";
    out << "Kp_theta=" << spin_kp_theta_->value() << "\n";
    out << "Ki_theta=" << spin_ki_theta_->value() << "\n";
    out << "Kd_theta=" << spin_kd_theta_->value() << "\n";
    out << "X_Tolerance=" << spin_x_tol_->value() << "\n";
    out << "Theta_Tolerance=" << spin_theta_tol_->value() << "\n";
    out << "track_width=" << spin_track_width_->value() << "\n";
    out << "TD_r=" << spin_td_r_->value() << "\n";
    out << "TD_h=" << spin_td_h_->value() << "\n";
    out << "Gear_Deadzone=" << spin_gear_dz_->value() << "\n";
    out << "Steer_Deadzone=" << spin_steer_dz_->value() << "\n\n";

    out << "# ===== 铲刀 PID (Tab3) =====\n";
    out << "Kp_Height_Up=" << spin_kp_h_up_->value() << "\n";
    out << "Kp_Height_Down=" << spin_kp_h_dn_->value() << "\n";
    out << "Ki_Height_Up=" << spin_ki_h_up_->value() << "\n";
    out << "Ki_Height_Down=" << spin_ki_h_dn_->value() << "\n";
    out << "Deadzone_Height=" << spin_dz_height_->value() << "\n";
    out << "I_MAX_Height=" << spin_imax_height_->value() << "\n";
    out << "Kp_Theta_Up=" << spin_kp_t_up_->value() << "\n";
    out << "Kp_Theta_Down=" << spin_kp_t_dn_->value() << "\n";
    out << "Ki_Theta_Up=" << spin_ki_t_up_->value() << "\n";
    out << "Ki_Theta_Down=" << spin_ki_t_dn_->value() << "\n";
    out << "Deadzone_Theta=" << spin_dz_theta_->value() << "\n";
    out << "I_MAX_Theta=" << spin_imax_theta_->value() << "\n\n";

    out << "# ===== 规划参数 (Tab5) =====\n";
    out << "Push_Length=" << spin_push_length_->value() << "\n";
    out << "Blade_Width=" << spin_blade_width_->value() << "\n";
    out << "Shift_Angle=" << spin_shift_angle_->value() << "\n";
    out << "Push_Heading=" << spin_push_heading_->value() << "\n";
    out << "V_Push=" << spin_v_push_->value() << "\n";
    out << "V_Reverse=" << spin_v_reverse_->value() << "\n";
    out << "Omega_Rotate=" << spin_omega_rotate_->value() << "\n";
    out << "X_Back_Set=" << spin_x_back_set_->value() << "\n";
    out << "Pos_Tolerance=" << spin_path_pos_tol_->value() << "\n";
    out << "Path_Theta_Tolerance=" << spin_path_theta_tol_->value() << "\n";
    out << "Tol_Run_X=" << spin_tol_run_x_->value() << "\n";
    out << "Tol_Run_Theta=" << spin_tol_run_theta_->value() << "\n";
    out << "Tol_Mold_Height=" << spin_tol_mold_height_->value() << "\n";
    out << "Speed_Gain_Risk=" << spin_speed_gain_risk_->value() << "\n";
    out << "Speed_Gain_Mold=" << spin_speed_gain_mold_->value() << "\n";
    out << "Mold_Limit=" << spin_mold_limit_->value() << "\n";
    out << "Overload_Trans=" << spin_overload_trans_->value() << "\n";
    out << "Overload_Vehicle=" << spin_overload_vehicle_->value() << "\n";
    out << "Overload_Angular=" << spin_overload_angular_->value() << "\n\n";

    out << "# ===== \u94F2\u5200\u63A7\u5236\u6A21\u5F0F (Tab5) =====\n";
    out << "Level_Mode=" << combo_level_mode_->selectedIndex() << "\n";
    out << "Target_Level_Height=" << spin_target_level_height_->value() << "\n";
    out << "Blade_Angle_Deg=" << spin_blade_angle_deg_->value() << "\n";
    out << "Slope_Start_Height=" << spin_slope_start_height_->value() << "\n";
    out << "Slope_Gradient=" << spin_slope_gradient_->value() << "\n\n";

    out << "# ===== \u6709\u5730\u56FE\u53C2\u6570 (Tab5) =====\n";
    out << "Map_Start_Corner=" << combo_map_start_corner_->selectedIndex() << "\n";
    out << "Map_Push_Heading=" << spin_map_push_heading_->selectedIndex() << "\n";

    out << "\n# ===== \u754C\u9762\u5E03\u5C40 =====\n";
    out << "Drawer_Open=" << (drawer_open_ ? 1 : 0) << "\n";
    out << "Drawer_Height=" << drawer_height_ << "\n";

    file.close();
    LOG_INFO("Config saved to: %s", path.toStdString().c_str());

    // 更新侧边栏配置文件显示
    side_config_path_->setText(QFileInfo(path).fileName());
    side_config_path_->setToolTip(path);

    // 保存后自动创建历史备份和变更日志
    saveHistory(path);

    QMessageBox::information(this, "\u4FDD\u5B58\u6210\u529F",
        QString("\u914D\u7F6E\u5DF2\u4FDD\u5B58\u5230:\n%1\n\n\u5386\u53F2\u5907\u4EFD\u5DF2\u521B\u5EFA").arg(path));
}

//==============================================================================
// 持久化: 从txt加载配置
//==============================================================================
void ManualControlWidget::loadConfig(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        LOG_ERROR("Cannot open file for reading: %s", path.toStdString().c_str());
        QMessageBox::warning(this, "\u52A0\u8F7D\u5931\u8D25",
            QString("\u65E0\u6CD5\u8BFB\u53D6\u6587\u4EF6:\n%1").arg(path));
        return;
    }

    // 建立 key → spinbox 映射
    QMap<QString, QDoubleSpinBox*> params;
    // Tab3 行走
    params["Kp_x"] = spin_kp_x_;
    params["Ki_x"] = spin_ki_x_;
    params["Kd_x"] = spin_kd_x_;
    params["Kp_theta"] = spin_kp_theta_;
    params["Ki_theta"] = spin_ki_theta_;
    params["Kd_theta"] = spin_kd_theta_;
    params["X_Tolerance"] = spin_x_tol_;
    params["Theta_Tolerance"] = spin_theta_tol_;
    params["track_width"] = spin_track_width_;
    params["TD_r"] = spin_td_r_;
    params["TD_h"] = spin_td_h_;
    params["Gear_Deadzone"] = spin_gear_dz_;
    params["Steer_Deadzone"] = spin_steer_dz_;
    // Tab3 铲刀
    params["Kp_Height_Up"] = spin_kp_h_up_;
    params["Kp_Height_Down"] = spin_kp_h_dn_;
    params["Ki_Height_Up"] = spin_ki_h_up_;
    params["Ki_Height_Down"] = spin_ki_h_dn_;
    params["Deadzone_Height"] = spin_dz_height_;
    params["I_MAX_Height"] = spin_imax_height_;
    params["Kp_Theta_Up"] = spin_kp_t_up_;
    params["Kp_Theta_Down"] = spin_kp_t_dn_;
    params["Ki_Theta_Up"] = spin_ki_t_up_;
    params["Ki_Theta_Down"] = spin_ki_t_dn_;
    params["Deadzone_Theta"] = spin_dz_theta_;
    params["I_MAX_Theta"] = spin_imax_theta_;
    // Tab5 规划
    params["Push_Length"] = spin_push_length_;
    params["Blade_Width"] = spin_blade_width_;
    params["Shift_Angle"] = spin_shift_angle_;
    params["Push_Heading"] = spin_push_heading_;
    params["V_Push"] = spin_v_push_;
    params["V_Reverse"] = spin_v_reverse_;
    params["Omega_Rotate"] = spin_omega_rotate_;
    params["X_Back_Set"] = spin_x_back_set_;
    params["Pos_Tolerance"] = spin_path_pos_tol_;
    params["Path_Theta_Tolerance"] = spin_path_theta_tol_;
    params["Tol_Run_X"] = spin_tol_run_x_;
    params["Tol_Run_Theta"] = spin_tol_run_theta_;
    params["Tol_Mold_Height"] = spin_tol_mold_height_;
    params["Speed_Gain_Risk"] = spin_speed_gain_risk_;
    params["Speed_Gain_Mold"] = spin_speed_gain_mold_;
    params["Mold_Limit"] = spin_mold_limit_;
    params["Overload_Trans"] = spin_overload_trans_;
    params["Overload_Vehicle"] = spin_overload_vehicle_;
    params["Overload_Angular"] = spin_overload_angular_;
    // 铲刀控制模式
    params["Target_Level_Height"] = spin_target_level_height_;
    params["Blade_Angle_Deg"] = spin_blade_angle_deg_;
    params["Slope_Start_Height"] = spin_slope_start_height_;
    params["Slope_Gradient"] = spin_slope_gradient_;

    int loaded = 0;
    bool load_drawer_open = false;
    int load_drawer_height = 420;
    bool has_layout = false;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;
        int eq = line.indexOf('=');
        if (eq <= 0) continue;
        QString key = line.left(eq).trimmed();
        QString val = line.mid(eq + 1).trimmed();
        if (params.contains(key)) {
            bool ok;
            double v = val.toDouble(&ok);
            if (ok) {
                params[key]->setValue(v);
                loaded++;
            }
        }
        if (key == "Drawer_Open") { load_drawer_open = (val.toInt() != 0); has_layout = true; }
        if (key == "Drawer_Height") { load_drawer_height = val.toInt(); }
        if (key == "Map_Start_Corner") { combo_map_start_corner_->setSelectedIndex(std::max(0, std::min(3, val.toInt()))); }
        if (key == "Map_Push_Heading") { spin_map_push_heading_->setSelectedIndex(std::max(0, std::min(3, val.toInt()))); }
        if (key == "Level_Mode") { combo_level_mode_->setSelectedIndex(std::max(0, std::min(1, val.toInt()))); }
    }
    file.close();

    // 恢复抽屉状态
    if (has_layout) {
        drawer_height_ = std::max(200, load_drawer_height);
        if (load_drawer_open && !drawer_open_) {
            toggleDrawer();
        } else if (!load_drawer_open && drawer_open_) {
            toggleDrawer();
        }
    }

    // 加载后更新 last_saved_params_ 快照 (为后续变更日志做基准)
    last_saved_params_.clear();
    for (auto it = params.begin(); it != params.end(); ++it)
        last_saved_params_[it.key()] = it.value()->value();

    // 更新侧边栏配置文件显示
    side_config_path_->setText(QFileInfo(path).fileName());
    side_config_path_->setToolTip(path);

    LOG_INFO("Config loaded from %s: %d parameters", path.toStdString().c_str(), loaded);
}

//==============================================================================
// ROS 回调
//==============================================================================
void ManualControlWidget::recordTopicTime(const QString& topic) {
    if(topic_diags_.contains(topic)) { topic_diags_[topic].count++; topic_diags_[topic].last_time_ms = diag_elapsed_.elapsed(); }
}
void ManualControlWidget::llaCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    lla_lat_=msg->point.y; lla_lon_=msg->point.x; lla_alt_=msg->point.z;
    recordTopicValue("/LLA", QString("%1, %2, %3m").arg(lla_lat_,0,'f',6).arg(lla_lon_,0,'f',6).arg(lla_alt_,0,'f',1));
}
void ManualControlWidget::angleCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    heading_deg_=msg->point.x*180.0/M_PI;
    recordTopicValue("/Angle_Heading", QString("%1\u00B0").arg(heading_deg_,0,'f',1));
}
void ManualControlWidget::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    recordTopicValue("/AHRS_IMU", QString("gyro=%.2f").arg(msg->angular_velocity.z));
}
void ManualControlWidget::walkStateCallback(const std_msgs::Float64::ConstPtr& msg) {
    walk_state_val_=msg->data;
    const char* ws_names[]={"stop","rot","drive","mid","end","prep","raise"};
    int ws=static_cast<int>(msg->data);
    recordTopicValue("/decision/walk_state", (ws>=0&&ws<=6)?ws_names[ws]:QString::number(ws));
}
void ManualControlWidget::decisionStatusCallback(const std_msgs::Float64::ConstPtr& msg) {
    decision_status_val_=msg->data;
    recordTopicValue("/Decision_Status", QString::number(msg->data,'f',0));
}
void ManualControlWidget::moldboardActualCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    if(msg->data.size()>=2){mold_right_mm_=msg->data[0];mold_left_mm_=msg->data[1];}
}
void ManualControlWidget::vRightCallback(const std_msgs::Float64::ConstPtr& msg) {
    v_right_val_=msg->data;
    recordTopicValue("/control/V_right", QString::number(msg->data,'f',1));
}
void ManualControlWidget::vLeftCallback(const std_msgs::Float64::ConstPtr& msg) {
    v_left_val_=msg->data;
    recordTopicValue("/control/V_left", QString::number(msg->data,'f',1));
}
void ManualControlWidget::terminalFlagCallback(const std_msgs::Float64::ConstPtr& msg) {
    terminal_flag_val_=msg->data;
    recordTopicValue("/control/terminal_flag", QString::number(msg->data,'f',0));
}
void ManualControlWidget::moldboardDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(msg->data.size()>=7){ mold_e_height_=msg->data[1]; mold_e_theta_=msg->data[2]; mold_u_height_=msg->data[5]; mold_u_theta_=msg->data[6]; }
    recordTopicValue("/control/moldboard_debug", QString("eH=%1 eT=%2 uH=%3 uT=%4")
        .arg(mold_e_height_,0,'f',1).arg(mold_e_theta_,0,'f',1).arg(mold_u_height_,0,'f',0).arg(mold_u_theta_,0,'f',0));
}
void ManualControlWidget::engineSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    engine_rpm_=msg->data;
    recordTopicValue("/Engine_Speed_Actual", QString("%1 rpm").arg(engine_rpm_,0,'f',0));
}
void ManualControlWidget::engineTorqueCallback(const std_msgs::Int8::ConstPtr& msg) { engine_torque_=msg->data; }
void ManualControlWidget::transSpeedCallback(const std_msgs::Int16::ConstPtr& msg) {
    trans_speed_=msg->data;
    recordTopicValue("/Transmission_Speed_Actual", QString::number(trans_speed_));
}
void ManualControlWidget::vehicleSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    vehicle_speed_=msg->data;
    recordTopicValue("/Vehicle_Speed_Vel", QString("%1 m/s").arg(vehicle_speed_,0,'f',2));
}
void ManualControlWidget::outputCurrentCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    for(int i=0;i<4&&i<(int)msg->data.size();i++) output_curr_[i]=msg->data[i];
}
void ManualControlWidget::imuPitchCallback(const std_msgs::Float64::ConstPtr& msg) {
    imu_pitch_deg_=msg->data;
    recordTopicValue("/IMU_Pitch", QString("%1\u00B0").arg(msg->data,0,'f',1));
}
void ManualControlWidget::imuRollCallback(const std_msgs::Float64::ConstPtr& msg) {
    imu_roll_deg_=msg->data;
    recordTopicValue("/IMU_Roll", QString("%1\u00B0").arg(msg->data,0,'f',1));
}

// Tab9 诊断快照
#include "tab9_diag_snapshot.inl"

// 铲刀PID自动整定
#include "tab3_autotune.inl"

// 行走PID自动整定
#include "tab3_walk_autotune.inl"

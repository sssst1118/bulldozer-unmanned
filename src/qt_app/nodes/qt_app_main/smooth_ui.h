/**
 * @file smooth_ui.h
 * @brief 特斯拉白色风格 UI 工具集
 * @author dozer-dev
 * @date 2026-04-23
 *
 * 依据《Qt5 丝滑UI优化规则-特斯拉白色风格》实现, 提供三个命名空间:
 *   Theme    —— 配色常量 (背景/文字/边框/功能色/滑块)
 *   Spacing  —— 间距与圆角常量 (特斯拉风偏大留白)
 *   SmoothUI —— 字体工厂与属性动画工具
 *
 * 用法: 在各 setStyleSheet / paintEvent 中直接引用 Theme::xxx,
 *       用 Theme::hex() 将 QColor 转成 "#RRGGBB" 塞进 QSS 字符串.
 *
 * Qt 兼容性: 仅使用 Qt 5 公共 API, 利用 C++17 inline variable 让 QColor
 *            可以直接放在 header 里不违反 ODR.
 */
#ifndef QT_APP_SMOOTH_UI_H
#define QT_APP_SMOOTH_UI_H

#include <QColor>
#include <QFont>
#include <QString>
#include <QObject>
#include <QWidget>
#include <QVariant>
#include <QByteArray>
#include <QEasingCurve>
#include <QPropertyAnimation>
#include <QAbstractAnimation>
#include <QTimer>
#include <QPushButton>
#include <QPointer>

//==============================================================================
// Theme — 特斯拉 + iOS 融合配色 (融合风规则一)
//   保留特斯拉: 克制配色 + 灰度文字层级 + 极浅边框
//   升级 iOS: 窗口底色 #F5F5F7 灰, 卡片纯白, iOS 功能色板, 纯黑主文字
//==============================================================================
namespace Theme {

// ----- 背景层次 -----
inline const QColor bgWindow   (245, 245, 247);   // #F5F5F7 窗口/桌面底色 (iOS 灰)
inline const QColor bgBase     (255, 255, 255);   // #FFFFFF 主背景 (保持向后兼容)
inline const QColor bgSurface  (255, 255, 255);   // #FFFFFF 卡片/面板 (iOS 风, 原为 #F9FAFB)
inline const QColor bgHover    (243, 243, 245);   // #F3F3F5 hover 微变
inline const QColor bgPressed  (233, 233, 237);   // #E9E9ED 按下
inline const QColor bgInput    (255, 255, 255);   // #FFFFFF 输入框

// ----- 毛玻璃专用 (供 GlassPanel 用) -----
inline QColor glassPanel()   { return QColor(255, 255, 255, 210); } // 主面板
inline QColor glassPopover() { return QColor(255, 255, 255, 235); } // 弹出层
inline QColor glassSegment() { return QColor(235, 235, 240, 180); } // 分段控制器底

// ----- 文字层次 (iOS: 纯黑主文字) -----
inline const QColor textPrimary  (  0,   0,   0); // #000000 iOS 纯黑
inline const QColor textSecondary(142, 142, 147); // #8E8E93 iOS 标准灰
inline const QColor textMuted    (174, 174, 178); // #AEAEB2
inline const QColor textDisabled (209, 209, 214); // #D1D1D6

// ----- 线条与边框 (iOS 比特斯拉更弱, 靠阴影分层) -----
inline const QColor borderDefault(224, 224, 229); // #E0E0E5 默认, 更浅
inline const QColor borderHover  (209, 209, 214); // #D1D1D6 hover
inline const QColor borderFocus  (  0, 122, 255); // #007AFF iOS 蓝

// ----- iOS 标准功能色板 -----
inline const QColor accentBlue   (  0, 122, 255); // #007AFF iOS 蓝
inline const QColor accentBlueBg (230, 240, 255); // #E6F0FF 蓝浅底
inline const QColor colorOk      ( 52, 199,  89); // #34C759 iOS 绿
inline const QColor colorOkBg    (232, 249, 236); // #E8F9EC 绿浅底
inline const QColor colorWarn    (255, 149,   0); // #FF9500 iOS 橙
inline const QColor colorWarnBg  (255, 248, 230); // #FFF8E6 橙浅底
inline const QColor colorError   (255,  59,  48); // #FF3B30 iOS 红
inline const QColor colorErrorBg (255, 238, 237); // #FFEEED 红浅底

// ----- 滑块/进度条 -----
inline const QColor sliderTrack  (229, 229, 234); // #E5E5EA iOS 灰
inline const QColor sliderFill   (  0, 122, 255); // #007AFF iOS 蓝

// ----- 开关 (iOS 风: 开启绿色, 不是蓝色) -----
inline const QColor toggleOff    (229, 229, 234); // #E5E5EA
inline const QColor toggleOn     ( 52, 199,  89); // #34C759

/// 把 QColor 转成 QSS 里直接可用的 "#RRGGBB" 字符串
inline QString hex(const QColor& c) { return c.name(QColor::HexRgb).toUpper(); }

/// 把带 alpha 的 QColor 转成 "rgba(r,g,b,a.aaa)" 字符串 (QSS 支持)
inline QString rgba(const QColor& c) {
    return QStringLiteral("rgba(%1,%2,%3,%4)")
        .arg(c.red()).arg(c.green()).arg(c.blue())
        .arg(c.alphaF(), 0, 'f', 3);
}

} // namespace Theme

//==============================================================================
// Spacing — 间距与圆角 (iOS 融合风)
//==============================================================================
namespace Spacing {
constexpr int panelPadding   = 20;   // iOS 标准 inset
constexpr int sectionGap     = 35;   // 大分组之间 (iOS 更大)
constexpr int groupGap       = 24;   // 控件组之间
constexpr int labelInputGap  = 8;    // 标签与输入框
constexpr int controlGap     = 12;   // 列表项之间
constexpr int cellHeight     = 44;   // iOS 标准触摸行高
constexpr int panelRadius    = 14;   // iOS 14+ 标准圆角
constexpr int inputRadius    = 10;   // 输入框圆角
constexpr int buttonRadius   = 12;   // 按钮圆角
constexpr int chipRadius     = 20;   // 标签/药丸圆角
constexpr int segmentRadius  = 9;    // 分段控制器圆角
constexpr int toggleWidth    = 51;   // iOS 标准开关宽度
constexpr int toggleHeight   = 31;   // iOS 标准开关高度
// 曲率参数: Squircle 曲线的阶数, 2 普通圆角, 4-5 iOS 超椭圆
constexpr double squircleN   = 4.5;
} // namespace Spacing

//==============================================================================
// SmoothUI — 字体工厂 + 动画工具 (规则二 + 规则五)
//==============================================================================
namespace SmoothUI {

// 注: Qt 5.13+ 才有 QFont::setFamilies(QStringList) 的多字体回退 API,
// Ubuntu 20.04 的 Qt 5.12 没有. 所以这里用 setFamily() + styleHint,
// Qt fontconfig 在首选字体不存在时会依据 styleHint 自动回退到系统首选
// (Linux 下等宽字体通常回退到 DejaVu Sans Mono, 无衬线回退到 DejaVu Sans).
// 首选字体名我们优先列跨平台可能有的 (JetBrains Mono / SF Pro), 不行就靠 styleHint.

/// 数值字体: 等宽, DemiBold, 微紧缩
inline QFont valueFont(int size = 14) {
    QFont f("JetBrains Mono");
    f.setStyleHint(QFont::Monospace);
    f.setPointSize(size);
    f.setWeight(QFont::DemiBold);
    f.setLetterSpacing(QFont::AbsoluteSpacing, -0.3);
    return f;
}

/// 标签字体: 无衬线, Normal
inline QFont labelFont(int size = 11) {
    QFont f("PingFang SC");
    f.setStyleHint(QFont::SansSerif);
    f.setPointSize(size);
    f.setWeight(QFont::Normal);
    return f;
}

/// 单位字体: 更小更浅, Light
inline QFont unitFont(int size = 9) {
    QFont f("PingFang SC");
    f.setStyleHint(QFont::SansSerif);
    f.setPointSize(size);
    f.setWeight(QFont::Light);
    return f;
}

/// 按钮字体: Medium
inline QFont buttonFont(int size = 12) {
    QFont f("PingFang SC");
    f.setStyleHint(QFont::SansSerif);
    f.setPointSize(size);
    f.setWeight(QFont::Medium);
    return f;
}

/// 在目标对象的某个 Qt property 上启动平滑动画, 自动取消同属性的旧动画
inline void animateProperty(QObject* target, const QByteArray& prop,
                            const QVariant& endValue, int durationMs = 250,
                            QEasingCurve::Type curve = QEasingCurve::OutCubic) {
    for (auto* child : target->children()) {
        if (auto* anim = qobject_cast<QPropertyAnimation*>(child)) {
            if (anim->propertyName() == prop &&
                anim->state() == QAbstractAnimation::Running) {
                anim->stop();
            }
        }
    }
    auto* anim = new QPropertyAnimation(target, prop);
    anim->setDuration(durationMs);
    anim->setEndValue(endValue);
    anim->setEasingCurve(curve);
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

/// 颜色动画快捷 (缓动 InOutQuad 更适合颜色过渡)
inline void animateColor(QWidget* w, const QByteArray& prop,
                         const QColor& to, int ms = 300) {
    animateProperty(w, prop, to, ms, QEasingCurve::InOutQuad);
}

/// 弹簧动画 (融合风规则四): 快速到达目标 → 轻微过冲 → 回弹稳定
/// 用 QEasingCurve::OutBack + overshoot. overshoot=1.0 ≈ iOS 标准弹簧,
/// 1.5 表现更弹, 0.8 更克制. 仅用于位移/缩放, 颜色永远用 animateColor.
inline void springAnimate(QObject* target, const QByteArray& prop,
                          const QVariant& endValue, int durationMs = 350,
                          double overshoot = 1.2) {
    for (auto* child : target->children()) {
        if (auto* anim = qobject_cast<QPropertyAnimation*>(child)) {
            if (anim->propertyName() == prop &&
                anim->state() == QAbstractAnimation::Running) {
                anim->stop();
            }
        }
    }
    auto* anim = new QPropertyAnimation(target, prop);
    anim->setDuration(durationMs);
    anim->setEndValue(endValue);
    QEasingCurve curve(QEasingCurve::OutBack);
    curve.setOvershoot(overshoot);
    anim->setEasingCurve(curve);
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

/// 平滑动画快捷 (OutCubic), 用于颜色/透明度/不需要弹性的过渡
inline void smoothAnimate(QObject* target, const QByteArray& prop,
                          const QVariant& endValue, int durationMs = 250) {
    animateProperty(target, prop, endValue, durationMs, QEasingCurve::OutCubic);
}

/// iOS 风输入错误抖动 (融合风补充篇·补充四):
/// 控件左右快速摇摆 3 次, 幅度递减, 配合红边框做验证失败反馈.
inline void shakeWidget(QWidget* widget, int amplitude = 6, int duration = 400) {
    if (!widget) return;
    auto* anim = new QPropertyAnimation(widget, "pos");
    anim->setDuration(duration);
    QPoint origin = widget->pos();
    anim->setKeyValueAt(0.00, origin);
    anim->setKeyValueAt(0.15, origin + QPoint(-amplitude, 0));
    anim->setKeyValueAt(0.30, origin + QPoint( amplitude, 0));
    anim->setKeyValueAt(0.45, origin + QPoint(static_cast<int>(-amplitude * 0.6), 0));
    anim->setKeyValueAt(0.60, origin + QPoint(static_cast<int>( amplitude * 0.6), 0));
    anim->setKeyValueAt(0.75, origin + QPoint(static_cast<int>(-amplitude * 0.3), 0));
    anim->setKeyValueAt(0.90, origin + QPoint(static_cast<int>( amplitude * 0.3), 0));
    anim->setKeyValueAt(1.00, origin);
    anim->setEasingCurve(QEasingCurve::Linear);   // 节奏由关键帧定义
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

/// 按钮"操作成功"反馈 (规则十): 按钮暂时变绿 + 显示成功文字, ms 后自动还原.
/// 用 dynamic property 保存原始 text/style, 防止重入时破坏状态.
inline void flashSuccess(QPushButton* btn,
                         const QString& okText = QStringLiteral("✓ 已发送"),
                         int ms = 1500) {
    if (!btn) return;
    // 已在闪烁则忽略, 避免多次点击覆盖 original
    if (btn->property("smoothui_flashing").toBool()) return;
    btn->setProperty("smoothui_original_text",  btn->text());
    btn->setProperty("smoothui_original_style", btn->styleSheet());
    btn->setProperty("smoothui_flashing", true);
    // 追加绿色成功样式, QSS cascade 里后写的覆盖前写的
    btn->setStyleSheet(btn->styleSheet() +
        "QPushButton{background:#16A34A;color:#FFFFFF;"
        "border:none;border-radius:8px;font-weight:500;padding:10px 24px;}"
        "QPushButton:hover{background:#15803D;}");
    btn->setText(okText);
    QPointer<QPushButton> guard(btn);    // 防按钮提前销毁
    QTimer::singleShot(ms, btn, [guard]() {
        if (!guard) return;
        guard->setStyleSheet(guard->property("smoothui_original_style").toString());
        guard->setText(guard->property("smoothui_original_text").toString());
        guard->setProperty("smoothui_flashing", false);
    });
}

} // namespace SmoothUI

#endif // QT_APP_SMOOTH_UI_H

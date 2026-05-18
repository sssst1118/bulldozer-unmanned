/**
 * @file glass_panel.h
 * @brief iOS 毛玻璃面板 (融合风规则五)
 * @author dozer-dev
 * @date 2026-05-06
 *
 * 提供两种毛玻璃实现:
 *   GlassPanel     —— 真实背景模糊 (QGraphicsBlurEffect), 视觉最贴近 iOS,
 *                     带脏标记缓存, resize/move 时才重抓+重模糊.
 *   FakeGlassPanel —— 半透明白底 + 顶部高光, 不抓背景不模糊, 性能极佳.
 *                     嵌入式/低性能设备首选, 视觉差异极小.
 *
 * 形状统一走 Squircle (规则三 iOS 超椭圆), 不用 drawRoundedRect.
 * 配色统一走 Theme::glassPanel/Popover/Segment (smooth_ui.h).
 *
 * 用法:
 *   auto* card = new FakeGlassPanel(this);
 *   card->setVariant(FakeGlassPanel::Popover);
 *
 *   auto* heavy = new GlassPanel(this);
 *   heavy->setBlurRadius(22);
 *   heavy->markDirty();   // 父级内容变化后通知重抓
 *
 * 阴影不在本类绘制, 由父级或包装层调用 ShadowUtils::drawCardShadow 完成,
 * 这样多层阴影可以独立缓存, 避免因模糊参数变化误失效阴影缓存.
 *
 * Qt 兼容性: 仅 Qt 5 公共 API.
 */
#ifndef QT_APP_GLASS_PANEL_H
#define QT_APP_GLASS_PANEL_H

#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QColor>
#include <QRegion>
#include <QResizeEvent>
#include <QMoveEvent>
#include <QPaintEvent>
#include <QLinearGradient>
#include <QGraphicsBlurEffect>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "smooth_ui.h"
#include "squircle.h"

//==============================================================================
// GlassPanel — 真实毛玻璃 (QGraphicsBlurEffect 实时模糊)
//   性能: paintEvent 抓取父级背景并做高斯模糊成本较高, 已用脏标记缓存;
//          仅在 resize/move/markDirty() 后才重算, 平时直接 drawPixmap.
//==============================================================================
class GlassPanel : public QWidget {
public:
    /// 变体: 决定半透明白底 alpha (Panel 主面板 / Popover 弹出层 / Segment 分段控制器)
    enum Variant { Panel, Popover, Segment };

    explicit GlassPanel(QWidget* parent = nullptr) : QWidget(parent) {
        setAttribute(Qt::WA_TranslucentBackground, true);
        setAttribute(Qt::WA_NoSystemBackground,    true);
    }

    void setVariant(Variant v)        { m_variant = v; update(); }
    void setBlurRadius(int radius)    { m_blurRadius = radius; m_dirty = true; update(); }
    void setRadius(int r)             { m_radius = r; update(); }
    /// 覆盖 variant 默认蒙版色 (传 invalid QColor 恢复默认)
    void setTintColor(const QColor& c){ m_tint = c; update(); }
    void setShowEdgeHighlight(bool e) { m_edgeHighlight = e; update(); }
    /// 父级内容变化后调用, 下一次 paint 重新抓取并模糊背景
    void markDirty()                  { m_dirty = true; update(); }

protected:
    void paintEvent(QPaintEvent*) override {
        if (m_grabbing) return;  // 防 grabBackground 触发递归 paint

        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        QRectF r = QRectF(rect());
        QPainterPath clip = Squircle::path(r, m_radius);

        // ---- 第一层: 模糊后的父级背景 ----
        if (m_dirty || m_cachedBlur.isNull() || m_cachedBlur.size() != size()) {
            m_cachedBlur = grabAndBlur();
            m_dirty = false;
        }
        if (!m_cachedBlur.isNull()) {
            p.save();
            p.setClipPath(clip);
            p.drawPixmap(0, 0, m_cachedBlur);
            p.restore();
        }

        // ---- 第二层: 半透明白色蒙版 (variant 决定 alpha) ----
        Squircle::draw(p, r, m_radius, QBrush(currentTint()));

        // ---- 第三层: 顶部 1px 高光 (iOS 毛玻璃上缘反光) ----
        if (m_edgeHighlight) {
            p.save();
            p.setClipPath(clip);
            QLinearGradient topLight(0, 0, 0, 1);
            topLight.setColorAt(0.0, QColor(255, 255, 255, 90));
            topLight.setColorAt(1.0, Qt::transparent);
            p.fillRect(0, 0, width(), 1, topLight);
            p.restore();
        }

        // ---- 第四层: 极细外边框 (iOS 玻璃边缘微光) ----
        QPen edge(QColor(255, 255, 255, 60));
        edge.setWidthF(0.5);
        p.setPen(edge);
        p.setBrush(Qt::NoBrush);
        p.drawPath(clip);
    }

    void resizeEvent(QResizeEvent* e) override {
        m_dirty = true;
        QWidget::resizeEvent(e);
    }
    void moveEvent(QMoveEvent* e) override {
        m_dirty = true;
        QWidget::moveEvent(e);
    }

private:
    QColor currentTint() const {
        if (m_tint.isValid()) return m_tint;
        switch (m_variant) {
            case Popover: return Theme::glassPopover();
            case Segment: return Theme::glassSegment();
            case Panel:
            default:      return Theme::glassPanel();
        }
    }

    /// 抓父级在本控件区域的内容, 然后高斯模糊
    QPixmap grabAndBlur() {
        QWidget* par = parentWidget();
        if (!par || size().isEmpty()) return QPixmap();

        // 临时关闭 updates 防 render() 自身被父级再次绘制造成视觉抖动
        m_grabbing = true;
        QPixmap snap(size());
        snap.fill(Qt::transparent);
        par->render(&snap, QPoint(),
                    QRegion(geometry()),
                    QWidget::DrawChildren | QWidget::IgnoreMask);
        m_grabbing = false;

        if (m_blurRadius <= 0) return snap;

        // 用 QGraphicsBlurEffect 做高斯模糊 (PerformanceHint 比 Quality 快得多)
        QGraphicsScene scene;
        auto* item = scene.addPixmap(snap);
        auto* blur = new QGraphicsBlurEffect;
        blur->setBlurRadius(m_blurRadius);
        blur->setBlurHints(QGraphicsBlurEffect::PerformanceHint);
        item->setGraphicsEffect(blur);

        QPixmap blurred(snap.size());
        blurred.fill(Qt::transparent);
        QPainter bp(&blurred);
        scene.render(&bp);
        return blurred;
    }

    Variant m_variant       = Panel;
    int     m_blurRadius    = 20;
    int     m_radius        = Spacing::panelRadius;
    QColor  m_tint;                  // invalid → 走 variant 默认
    bool    m_edgeHighlight = true;
    QPixmap m_cachedBlur;
    bool    m_dirty         = true;
    bool    m_grabbing      = false; // re-entry 守卫
};

//==============================================================================
// FakeGlassPanel — 轻量伪毛玻璃 (推荐默认)
//   不抓背景不模糊, 仅靠半透明白底 + 顶部高光 + 极弱边框模拟 iOS 玻璃质感.
//   嵌入式/触摸屏/低性能场景首选, 视觉差异极小, 帧率成本几乎为零.
//==============================================================================
class FakeGlassPanel : public QWidget {
public:
    enum Variant { Panel, Popover, Segment };

    explicit FakeGlassPanel(QWidget* parent = nullptr) : QWidget(parent) {
        setAttribute(Qt::WA_TranslucentBackground, true);
        setAttribute(Qt::WA_NoSystemBackground,    true);
    }

    void setVariant(Variant v)        { m_variant = v; update(); }
    void setRadius(int r)             { m_radius = r; update(); }
    void setTintColor(const QColor& c){ m_tint = c; update(); }
    void setShowTopHighlight(bool e)  { m_topHighlight = e; update(); }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        QRectF r = QRectF(rect());
        QPainterPath clip = Squircle::path(r, m_radius);

        // 半透明白底 (variant 决定 alpha)
        Squircle::draw(p, r, m_radius, QBrush(currentTint()));

        // 顶部 1px 高光线 (iOS 玻璃上缘反光)
        if (m_topHighlight) {
            p.save();
            p.setClipPath(clip);
            QLinearGradient topLight(0, 0, 0, 1);
            topLight.setColorAt(0.0, QColor(255, 255, 255, 80));
            topLight.setColorAt(1.0, Qt::transparent);
            p.fillRect(0, 0, width(), 1, topLight);
            p.restore();
        }

        // 极弱外边框 (alpha 8, iOS 风靠阴影分层而非边框)
        QPen edge(QColor(0, 0, 0, 8));
        edge.setWidthF(0.5);
        p.setPen(edge);
        p.setBrush(Qt::NoBrush);
        p.drawPath(clip);
    }

private:
    QColor currentTint() const {
        if (m_tint.isValid()) return m_tint;
        switch (m_variant) {
            case Popover: return Theme::glassPopover();
            case Segment: return Theme::glassSegment();
            case Panel:
            default:      return Theme::glassPanel();
        }
    }

    Variant m_variant      = Panel;
    int     m_radius       = Spacing::panelRadius;
    QColor  m_tint;
    bool    m_topHighlight = true;
};

#endif // QT_APP_GLASS_PANEL_H

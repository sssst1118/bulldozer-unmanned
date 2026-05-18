/**
 * @file shadow_utils.h
 * @brief iOS 风多层阴影绘制工具 (融合风规则六)
 * @author dozer-dev
 * @date 2026-04-23
 *
 * iOS 用多层阴影 (而非边框) 建立纵深层级:
 *   drawCardShadow    — 卡片标准阴影 (两层叠加)
 *   drawPopoverShadow — 弹出层重阴影 (三层叠加)
 *   drawShadow        — 单层阴影底层工具
 *
 * 性能: 每层阴影都要做一次 QGraphicsBlurEffect 渲染, 成本可观.
 *       本模块用 QPixmapCache 按 (w,h,radius,blur,color) 缓存阴影 pixmap,
 *       固定尺寸的控件只算一次, 每帧 drawPixmap 即可.
 *       - resize 时缓存 miss 自动重算.
 *       - 阴影颜色变化 (如 hover 加深) 会产生新缓存 key, 不互相污染.
 */
#ifndef QT_APP_SHADOW_UTILS_H
#define QT_APP_SHADOW_UTILS_H

#include <QPainter>
#include <QPixmap>
#include <QPixmapCache>
#include <QRect>
#include <QColor>
#include <QGraphicsBlurEffect>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "squircle.h"

namespace ShadowUtils {

namespace detail {

/// 实际渲染一层阴影到 pixmap (带 Squircle 超椭圆形状)
inline QPixmap renderShadowPixmap(int width, int height, int radius,
                                  const QColor& color, int blurRadius) {
    int pad = qMax(1, blurRadius * 2);
    QPixmap pm(width + pad * 2, height + pad * 2);
    pm.fill(Qt::transparent);

    // 先画一个 Squircle 形状的纯色矩形
    {
        QPainter sp(&pm);
        sp.setRenderHint(QPainter::Antialiasing, true);
        Squircle::draw(sp, QRectF(pad, pad, width, height),
                       radius, QBrush(color));
    }

    // 用 QGraphicsBlurEffect 做高斯模糊
    if (blurRadius > 0) {
        QGraphicsScene scene;
        auto* item = scene.addPixmap(pm);
        auto* blur = new QGraphicsBlurEffect;
        blur->setBlurRadius(blurRadius);
        blur->setBlurHints(QGraphicsBlurEffect::QualityHint);
        item->setGraphicsEffect(blur);

        QPixmap blurred(pm.size());
        blurred.fill(Qt::transparent);
        QPainter bp(&blurred);
        scene.render(&bp);
        return blurred;
    }
    return pm;
}

inline QString cacheKey(int w, int h, int radius,
                        const QColor& color, int blurRadius) {
    return QStringLiteral("shadow_%1x%2_r%3_c%4_b%5")
        .arg(w).arg(h).arg(radius)
        .arg(color.rgba(), 0, 16).arg(blurRadius);
}

} // namespace detail

/// 画一层阴影, 自动走 QPixmapCache
inline void drawShadow(QPainter& p, const QRect& rect, int radius,
                       const QColor& color, int offsetX, int offsetY,
                       int blurRadius) {
    QPixmap cached;
    const QString key = detail::cacheKey(rect.width(), rect.height(),
                                         radius, color, blurRadius);
    if (!QPixmapCache::find(key, &cached)) {
        cached = detail::renderShadowPixmap(rect.width(), rect.height(),
                                            radius, color, blurRadius);
        QPixmapCache::insert(key, cached);
    }
    int pad = qMax(1, blurRadius * 2);
    p.drawPixmap(rect.x() - pad + offsetX,
                 rect.y() - pad + offsetY, cached);
}

/// iOS 卡片标准阴影 (两层叠加): 紧贴边缘 + 柔和扩散
inline void drawCardShadow(QPainter& p, const QRect& rect, int radius = 14) {
    drawShadow(p, rect, radius, QColor(0, 0, 0,  8),  0, 0,  1);
    drawShadow(p, rect, radius, QColor(0, 0, 0, 12),  0, 2,  8);
}

/// iOS 弹出层阴影 (三层叠加): 更重, 用于 popover/modal
inline void drawPopoverShadow(QPainter& p, const QRect& rect, int radius = 14) {
    drawShadow(p, rect, radius, QColor(0, 0, 0,  6),  0, 0,   1);
    drawShadow(p, rect, radius, QColor(0, 0, 0, 10),  0, 4,  16);
    drawShadow(p, rect, radius, QColor(0, 0, 0, 15),  0, 12, 40);
}

} // namespace ShadowUtils

#endif // QT_APP_SHADOW_UTILS_H

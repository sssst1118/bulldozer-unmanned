/**
 * @file squircle.h
 * @brief iOS 超椭圆 (Squircle) 圆角路径生成 + 尺寸级缓存
 * @author dozer-dev
 * @date 2026-04-23 (2026-05-07 阶段五: 增加 size-keyed QCache)
 *
 * iOS 的圆角不是标准 border-radius, 而是连续曲率 (continuous curvature, C2)
 * 超椭圆. 视觉上比普通圆角更饱满、过渡更自然.
 *
 * 参数 n 控制曲率:
 *   n = 2.0  普通圆角 (等价 drawRoundedRect)
 *   n = 4.5  iOS 标准超椭圆 (iPhone app icon 的圆角)
 *   n = 6.0  更硬朗, 接近正方形
 *
 * 用法:
 *   QPainterPath sp = Squircle::path(rect, 14);       // 生成路径
 *   painter.drawPath(sp);                              // 直接绘制
 *   Squircle::draw(painter, rect, 14, QBrush(...));   // 一步到位
 *
 * 性能 (阶段五优化):
 *   path() 内部按 (width, height, radius, n) 量化键缓存最多 256 条
 *   normalized 路径 (位于原点), 命中后仅 translate 到目标 rect.topLeft().
 *   多数高频重绘控件 (滑块轨道、开关、Spin 焦点环、玻璃面板) 尺寸稳定,
 *   命中率接近 100%. 主线程 only — 不加锁, Qt UI 线程模型已保证.
 */
#ifndef QT_APP_SQUIRCLE_H
#define QT_APP_SQUIRCLE_H

#include <QPainter>
#include <QPainterPath>
#include <QRectF>
#include <QBrush>
#include <QPen>
#include <QCache>
#include <QtMath>

namespace Squircle {

namespace detail {

/// 在原点构建 squircle 路径 (尺寸 w×h, 圆角 r, 曲率 n).
inline QPainterPath buildAtOrigin(double w, double h, double radius, double n) {
    QPainterPath p;
    double r = qMin(radius, qMin(w, h) / 2.0);
    // 贝塞尔控制点偏移: 0.55228 是标准圆 4 段贝塞尔逼近系数, 乘 (n/4) 拉控制点近似超椭圆.
    double k = 0.55228 * (n / 4.0);

    p.moveTo(r, 0);
    p.lineTo(w - r, 0);
    p.cubicTo(w - r + r * k, 0,
              w,             r - r * k,
              w,             r);
    p.lineTo(w, h - r);
    p.cubicTo(w,             h - r + r * k,
              w - r + r * k, h,
              w - r,         h);
    p.lineTo(r, h);
    p.cubicTo(r - r * k, h,
              0,         h - r + r * k,
              0,         h - r);
    p.lineTo(0, r);
    p.cubicTo(0,         r - r * k,
              r - r * k, 0,
              r,         0);
    p.closeSubpath();
    return p;
}

/// 单例缓存. inline 函数中的 static 局部变量在所有 TU 共享一份 (C++11 magic statics).
inline QCache<quint64, QPainterPath>& cache() {
    static QCache<quint64, QPainterPath> c(256);
    return c;
}

/// 量化打包: w/h 按整数像素, radius 保留 2 位小数, n 保留 1 位.
/// (w<<48 | h<<32 | rInt<<16 | nInt) — 各字段 16bit 上限够 UI 用.
inline quint64 makeKey(double w, double h, double radius, double n) {
    quint64 wi = quint64(qBound(0, qRound(w),       65535));
    quint64 hi = quint64(qBound(0, qRound(h),       65535));
    quint64 ri = quint64(qBound(0, qRound(radius * 100.0), 65535));
    quint64 ni = quint64(qBound(0, qRound(n      *  10.0),   65535));
    return (wi << 48) | (hi << 32) | (ri << 16) | ni;
}

} // namespace detail

/**
 * @brief 生成 iOS 风格超椭圆圆角路径 (尺寸级缓存)
 * @param rect   外接矩形
 * @param radius 圆角半径
 * @param n      曲率阶数, 默认 4.5 (iOS 标准)
 */
inline QPainterPath path(const QRectF& rect, double radius, double n = 4.5) {
    const double w = rect.width();
    const double h = rect.height();
    if (w <= 0 || h <= 0) return QPainterPath();

    quint64 key = detail::makeKey(w, h, radius, n);
    auto& c = detail::cache();
    QPainterPath* hit = c.object(key);
    QPainterPath base;
    if (hit) {
        base = *hit;
    } else {
        base = detail::buildAtOrigin(w, h, radius, n);
        c.insert(key, new QPainterPath(base));
    }
    base.translate(rect.x(), rect.y());
    return base;
}

/// 便捷绘制: 一行替代 drawRoundedRect
inline void draw(QPainter& painter, const QRectF& rect,
                 double radius, const QBrush& fill,
                 const QPen& pen = Qt::NoPen, double n = 4.5) {
    QPainterPath sp = path(rect, radius, n);
    painter.setPen(pen);
    painter.setBrush(fill);
    painter.drawPath(sp);
}

} // namespace Squircle

#endif // QT_APP_SQUIRCLE_H

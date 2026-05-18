/**
 * @file gesture_handler.h
 * @brief iOS 风格触摸/鼠标手势识别 (header-only, 无 Q_OBJECT)
 * @author dozer-dev
 * @date 2026-05-07 (阶段五)
 *
 * 在任意 QWidget 上识别 swipe(左/右/上/下) / tap / long-press 三类手势,
 * 通过 std::function 回调上报. 不消费事件 (eventFilter 永远 return false),
 * 因此原控件的 click / drag 行为不会被打断.
 *
 * 用法:
 *   GestureHandler::install(myWidget)
 *       ->setOnSwipe([this](GestureHandler::Direction d) {
 *             if (d == GestureHandler::DirLeft)  ...
 *         })
 *       ->setOnLongPress([this](const QPoint& p) { ... });
 *
 * 设计:
 *   - 不写 Q_OBJECT (无信号/槽/属性), 因此无需进 MOC, 无需改 CMakeLists.
 *   - 仍继承 QObject (eventFilter 必需), 父对象 = target widget, 自动跟随销毁.
 *   - tap_max_move=10px 阈值兼容鼠标抖动; swipe_min=60px / max_dur=500ms 是
 *     iOS 系统 swipe 默认的近似值.
 */
#ifndef QT_APP_GESTURE_HANDLER_H
#define QT_APP_GESTURE_HANDLER_H

#include <QObject>
#include <QWidget>
#include <QEvent>
#include <QMouseEvent>
#include <QTimer>
#include <QDateTime>
#include <QtMath>
#include <functional>

class GestureHandler : public QObject {
public:
    enum Direction { DirLeft, DirRight, DirUp, DirDown };
    using SwipeCb = std::function<void(Direction)>;
    using PointCb = std::function<void(const QPoint&)>;

    /// 在 target 上安装手势识别. 返回的 handler 父对象 = target, 自动销毁.
    static GestureHandler* install(QWidget* target) {
        if (!target) return nullptr;
        auto* h = new GestureHandler(target);
        target->installEventFilter(h);
        return h;
    }

    GestureHandler* setOnSwipe(SwipeCb cb)     { swipe_cb_      = std::move(cb); return this; }
    GestureHandler* setOnTap(PointCb cb)       { tap_cb_        = std::move(cb); return this; }
    GestureHandler* setOnLongPress(PointCb cb) { long_press_cb_ = std::move(cb); return this; }

    GestureHandler* setSwipeMinDistance(int px) { swipe_min_        = px; return this; }
    GestureHandler* setSwipeMaxDuration(int ms) { swipe_max_dur_    = ms; return this; }
    GestureHandler* setLongPressDelay(int ms)   { long_press_delay_ = ms; return this; }
    GestureHandler* setTapMaxMove(int px)       { tap_max_move_     = px; return this; }

protected:
    bool eventFilter(QObject* obj, QEvent* e) override {
        Q_UNUSED(obj);
        switch (e->type()) {
        case QEvent::MouseButtonPress: {
            auto* me = static_cast<QMouseEvent*>(e);
            if (me->button() != Qt::LeftButton) break;
            press_pos_      = me->pos();
            press_time_ms_  = QDateTime::currentMSecsSinceEpoch();
            tracking_       = true;
            moved_too_far_  = false;
            if (long_press_cb_) long_press_timer_->start(long_press_delay_);
            break;
        }
        case QEvent::MouseMove: {
            if (!tracking_) break;
            auto* me = static_cast<QMouseEvent*>(e);
            if ((me->pos() - press_pos_).manhattanLength() > tap_max_move_) {
                moved_too_far_ = true;
                long_press_timer_->stop();
            }
            break;
        }
        case QEvent::MouseButtonRelease: {
            if (!tracking_) break;
            auto* me = static_cast<QMouseEvent*>(e);
            if (me->button() != Qt::LeftButton) break;
            tracking_ = false;
            long_press_timer_->stop();
            const QPoint dp = me->pos() - press_pos_;
            const qint64 dur_ms = QDateTime::currentMSecsSinceEpoch() - press_time_ms_;
            const int adx = qAbs(dp.x());
            const int ady = qAbs(dp.y());
            const bool is_swipe =
                (adx >= swipe_min_ || ady >= swipe_min_) && dur_ms <= swipe_max_dur_;
            if (is_swipe && swipe_cb_) {
                Direction d = (adx >= ady)
                    ? (dp.x() > 0 ? DirRight : DirLeft)
                    : (dp.y() > 0 ? DirDown  : DirUp);
                swipe_cb_(d);
            } else if (!moved_too_far_ && tap_cb_) {
                tap_cb_(press_pos_);
            }
            break;
        }
        // 鼠标离开窗口/失焦 → 取消跟踪, 防止悬停态长按
        case QEvent::Leave:
        case QEvent::FocusOut:
            if (tracking_) {
                tracking_ = false;
                long_press_timer_->stop();
            }
            break;
        default:
            break;
        }
        return false;  // 永不消费, 让原控件继续接收事件
    }

private:
    explicit GestureHandler(QWidget* parent) : QObject(parent) {
        long_press_timer_ = new QTimer(this);
        long_press_timer_->setSingleShot(true);
        connect(long_press_timer_, &QTimer::timeout, this, [this]() {
            if (!tracking_ || moved_too_far_) return;
            if (long_press_cb_) long_press_cb_(press_pos_);
            tracking_ = false;  // 长按已触发, 不再回退为 tap
        });
    }

    QPoint  press_pos_;
    qint64  press_time_ms_ = 0;
    bool    tracking_      = false;
    bool    moved_too_far_ = false;
    QTimer* long_press_timer_ = nullptr;

    int swipe_min_        = 60;   // px
    int swipe_max_dur_    = 500;  // ms
    int long_press_delay_ = 600;  // ms (iOS 标准 ~500-700)
    int tap_max_move_     = 10;   // px

    SwipeCb swipe_cb_;
    PointCb tap_cb_;
    PointCb long_press_cb_;
};

#endif // QT_APP_GESTURE_HANDLER_H

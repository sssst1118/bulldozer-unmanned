/**
 * @file grid_map_widget.cpp
 * @brief 栅格地图可视化 — 三并排 (原始地图 / 规划路径 / 实时轨迹)
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "grid_map_widget.h"
#include <boost/function.hpp>
#include <algorithm>

GridMapWidget::GridMapWidget(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(450, 200);
    sub_map_  = nh.subscribe("/occupancy_grid", 1, &GridMapWidget::mapCallback, this);
    sub_loc_  = nh.subscribe("/Navigate_location", 1, &GridMapWidget::locationCallback, this);
    sub_path_ = nh.subscribe("/Path", 1, &GridMapWidget::pathCallback, this);
    sub_dir_  = nh.subscribe("/Direction", 1, &GridMapWidget::directionCallback, this);
    sub_full_path_ = nh.subscribe("/decision/full_path", 1, &GridMapWidget::fullPathCallback, this);
    sub_wp_index_  = nh.subscribe<std_msgs::Float64>("/decision/waypoint_index", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                QMutexLocker lock(&mutex_);
                current_wp_index_ = static_cast<int>(msg->data);
                update();
            }));
}

// ===================== ROS 回调 =====================
void GridMapWidget::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    QMutexLocker lock(&mutex_);
    map_rows_ = static_cast<int>(msg->info.height);
    map_cols_ = static_cast<int>(msg->info.width);
    resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_data_.assign(msg->data.begin(), msg->data.end());
    update();
}
void GridMapWidget::locationCallback(const geometry_msgs::Point::ConstPtr& msg) {
    QMutexLocker lock(&mutex_);
    // /Navigate_location 是 ENU 坐标, 转成栅格行列
    if (resolution_ > 0) {
        veh_col_ = (msg->x - map_origin_x_) / resolution_;
        veh_row_ = (msg->y - map_origin_y_) / resolution_;
    }
    if (veh_row_ >= 0 && veh_col_ >= 0) {
        if (trail_.empty() ||
            std::abs(trail_.back().first - veh_row_) > 0.3 ||
            std::abs(trail_.back().second - veh_col_) > 0.3) {
            trail_.push_back({veh_row_, veh_col_});
            if (static_cast<int>(trail_.size()) > kMaxTrail)
                trail_.erase(trail_.begin());
        }
    }
    update();
}
void GridMapWidget::pathCallback(const geometry_msgs::Point::ConstPtr& msg) {
    QMutexLocker lock(&mutex_);
    path_theta_ = msg->x; path_dist_ = msg->y;
    update();
}
void GridMapWidget::directionCallback(const std_msgs::Float64::ConstPtr& msg) {
    QMutexLocker lock(&mutex_);
    direction_ = msg->data;
    update();
}
void GridMapWidget::fullPathCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    QMutexLocker lock(&mutex_);
    planned_path_.clear();
    for (size_t i = 0; i + 2 < msg->data.size(); i += 3)
        planned_path_.push_back({msg->data[i], msg->data[i+1], msg->data[i+2]});
    update();
}

// ===================== 主绘图: 三并排 =====================
void GridMapWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QMutexLocker lock(&mutex_);

    int W = width(), H = height();
    int gap = 6;            // 地图之间间距
    int panelW = (W - gap * 2) / 3;  // 每张地图宽度
    int titleH = 22;        // 标题高度

    // 没有地图数据
    if (map_rows_ <= 0 || map_cols_ <= 0 || map_data_.empty()) {
        p.setPen(QColor(150,150,150));
        p.setFont(QFont("", 11));
        p.drawText(rect(), Qt::AlignCenter, "等待栅格地图数据...\n/occupancy_grid");
        return;
    }

    // 计算每张地图内格子大小
    int margin = 4;
    int drawH = H - titleH - margin * 2;
    double cell_w = static_cast<double>(panelW - margin * 2) / map_cols_;
    double cell_h = static_cast<double>(drawH) / map_rows_;
    double cell = std::min(cell_w, cell_h);
    if (cell < 1) cell = 1;

    // 三张地图的标题和叠加内容
    const char* titles[] = {"原始地图", "规划路径", "实时轨迹"};
    QColor titleColors[] = {QColor(100,100,100), QColor(0,120,255), QColor(0,180,80)};

    for (int vi = 0; vi < 3; ++vi) {
        int px = vi * (panelW + gap);  // 面板左边x

        // 面板背景 + 圆角
        QRect panelRect(px, 0, panelW, H);
        p.setPen(QPen(QColor(220,220,220), 1));
        p.setBrush(QColor(252,252,252));
        p.drawRoundedRect(panelRect, 8, 8);

        // 标题
        QRect titleRect(px, 2, panelW, titleH);
        p.setPen(titleColors[vi]);
        p.setFont(QFont("", 9, QFont::Bold));
        p.drawText(titleRect, Qt::AlignCenter, titles[vi]);

        // 地图居中偏移
        double ox = px + margin + (panelW - margin*2 - cell * map_cols_) / 2.0;
        double oy = titleH + margin + (drawH - cell * map_rows_) / 2.0;

        // 所有视图画底图
        drawMapCells(p, ox, oy, cell);
        if (cell >= 4) drawGrid(p, ox, oy, cell);

        // 叠加层
        if (vi == 1) {
            // 规划路径视图
            drawPlannedPath(p, ox, oy, cell);
        } else if (vi == 2) {
            // 实时轨迹视图
            drawTrail(p, ox, oy, cell);
        }

        // 车辆 (所有视图)
        drawVehicle(p, ox, oy, cell);

        // 当前执行段红色箭头 (只在规划路径视图)
        if (vi == 1) drawCurrentSegment(p, ox, oy, cell);

        // 地图尺寸标注 (只在第一张)
        if (vi == 0) {
            p.setPen(QColor(150,150,150));
            p.setFont(QFont("", 7));
            p.drawText(QPointF(ox, oy + map_rows_ * cell + 10),
                       QString("%1×%2 | %3m/格").arg(map_rows_).arg(map_cols_).arg(resolution_,0,'f',1));
        }
        // 路径进度 (规划视图)
        if (vi == 1 && !planned_path_.empty()) {
            p.setPen(QColor(0,120,255));
            p.setFont(QFont("", 7));
            p.drawText(QPointF(px + margin, oy + map_rows_ * cell + 10),
                       QString("WP: %1/%2").arg(current_wp_index_).arg(static_cast<int>(planned_path_.size())));
        }
        // 轨迹点数 (轨迹视图)
        if (vi == 2 && !trail_.empty()) {
            p.setPen(QColor(0,180,80));
            p.setFont(QFont("", 7));
            p.drawText(QPointF(px + margin, oy + map_rows_ * cell + 10),
                       QString("轨迹: %1点").arg(static_cast<int>(trail_.size())));
        }
    }
}

// ===================== 栅格底图 =====================
// OccupancyGrid: row=0 在底部(南), 但屏幕 y=0 在顶部
// 所以画的时候要翻转: screen_y = oy + (map_rows-1-r) * cell
void GridMapWidget::drawMapCells(QPainter& p, double ox, double oy, double cell) {
    for (int r = 0; r < map_rows_; ++r) {
        int screen_r = map_rows_ - 1 - r;  // 翻转: row=0画在最下面
        for (int c = 0; c < map_cols_; ++c) {
            int idx = r * map_cols_ + c;
            int8_t val = (idx < static_cast<int>(map_data_.size())) ? map_data_[idx] : -1;
            QColor color;
            if (val == 0)       color = QColor(245, 245, 245);
            else if (val > 0)   color = QColor(55, 55, 55);
            else                color = QColor(180, 180, 180);
            p.fillRect(QRectF(ox + c*cell, oy + screen_r*cell, cell, cell), color);
        }
    }
}

// ===================== 网格线 =====================
void GridMapWidget::drawGrid(QPainter& p, double ox, double oy, double cell) {
    p.setPen(QPen(QColor(210,210,210), 0.3));
    for (int r = 0; r <= map_rows_; ++r) {
        double y = oy + r * cell;
        p.drawLine(QPointF(ox, y), QPointF(ox + map_cols_*cell, y));
    }
    for (int c = 0; c <= map_cols_; ++c) {
        double x = ox + c * cell;
        p.drawLine(QPointF(x, oy), QPointF(x, oy + map_rows_*cell));
    }
}

// ===================== 规划路径 (蓝色) =====================
// planned_path_ 存储的是栅格坐标: x=row, y=col, dir=方向
void GridMapWidget::drawPlannedPath(QPainter& p, double ox, double oy, double cell) {
    if (planned_path_.empty() || map_rows_ <= 0) return;

    // 栅格(row,col) → 像素, row翻转(row=0在底部)
    auto toPixel = [&](double row, double col) -> QPointF {
        double sr = map_rows_ - 1 - row;  // 翻转
        return QPointF(ox + col * cell + cell/2, oy + sr * cell + cell/2);
    };

    // 路径线段: 三种颜色
    //   dir=1.0 推土 → 绿色实线
    //   dir=2.0 空驶 → 蓝色虚线
    //   dir<0   倒车 → 灰色点线
    for (size_t i = 1; i < planned_path_.size(); ++i) {
        QPointF p1 = toPixel(planned_path_[i-1].x, planned_path_[i-1].y);
        QPointF p2 = toPixel(planned_path_[i].x, planned_path_[i].y);
        double d = planned_path_[i].dir;
        if (d < 0)
            p.setPen(QPen(QColor(180,180,180,160), std::max(cell*0.2, 1.0), Qt::DotLine));
        else if (d > 1.5)
            p.setPen(QPen(QColor(0,120,255,180), std::max(cell*0.2, 1.0), Qt::DashLine));
        else
            p.setPen(QPen(QColor(0,180,80,220), std::max(cell*0.25, 1.5), Qt::SolidLine));
        p.drawLine(p1, p2);
    }

    // 路径点
    double dotR = std::max(cell * 0.25, 2.5);
    for (size_t i = 0; i < planned_path_.size(); ++i) {
        QPointF pt = toPixel(planned_path_[i].x, planned_path_[i].y);
        bool isCurrent = (static_cast<int>(i) == current_wp_index_);
        bool isDone    = (static_cast<int>(i) < current_wp_index_);

        QColor c = isCurrent ? QColor(255,165,0) : isDone ? QColor(100,200,100) : QColor(0,120,255);
        p.setPen(Qt::NoPen);
        p.setBrush(c);
        p.drawEllipse(pt, dotR, dotR);

        // 编号 (每5个或当前点)
        if (i % 5 == 0 || isCurrent) {
            p.setPen(QColor(40,40,40));
            p.setFont(QFont("", std::max(static_cast<int>(dotR*1.2), 6)));
            p.drawText(pt + QPointF(dotR+1, -1), QString::number(i));
        }
    }

    // 起点 S (红), 终点 E (绿)
    double mkSz = std::max(dotR * 1.5, 4.0);
    if (!planned_path_.empty()) {
        QPointF sp = toPixel(planned_path_[0].x, planned_path_[0].y);
        p.setPen(QPen(Qt::red, 1)); p.setBrush(QColor(255,0,0,180));
        p.drawRect(QRectF(sp.x()-mkSz, sp.y()-mkSz, mkSz*2, mkSz*2));
        p.setPen(Qt::white); p.setFont(QFont("", std::max(static_cast<int>(mkSz), 7), QFont::Bold));
        p.drawText(sp + QPointF(-mkSz*0.3, mkSz*0.4), "S");
    }
    if (planned_path_.size() > 1) {
        QPointF ep = toPixel(planned_path_.back().x, planned_path_.back().y);
        p.setPen(QPen(QColor(0,160,0), 1)); p.setBrush(QColor(0,160,0,180));
        p.drawRect(QRectF(ep.x()-mkSz, ep.y()-mkSz, mkSz*2, mkSz*2));
        p.setPen(Qt::white); p.setFont(QFont("", std::max(static_cast<int>(mkSz), 7), QFont::Bold));
        p.drawText(ep + QPointF(-mkSz*0.3, mkSz*0.4), "E");
    }
}

// ===================== 实时轨迹 (绿色渐变) =====================
void GridMapWidget::drawTrail(QPainter& p, double ox, double oy, double cell) {
    if (trail_.size() < 2) return;
    for (size_t i = 1; i < trail_.size(); ++i) {
        int alpha = 40 + static_cast<int>(180.0 * i / trail_.size());
        double lw = std::max(cell * 0.35, 1.5);
        p.setPen(QPen(QColor(0, 200, 80, alpha), lw));
        double sr1 = map_rows_ - 1 - trail_[i-1].first;
        double sr2 = map_rows_ - 1 - trail_[i].first;
        p.drawLine(
            QPointF(ox + trail_[i-1].second * cell + cell/2, oy + sr1 * cell + cell/2),
            QPointF(ox + trail_[i].second * cell + cell/2,   oy + sr2 * cell + cell/2));
    }
}

// ===================== 当前路径段 (红色箭头) =====================
void GridMapWidget::drawCurrentSegment(QPainter& p, double ox, double oy, double cell) {
    if (veh_row_ < 0 || path_dist_ < 0.01) return;
    double vx = ox + veh_col_ * cell + cell/2;
    double vy = oy + (map_rows_ - 1 - veh_row_) * cell + cell/2;  // 翻转
    double rad = -path_theta_ * M_PI / 180.0;
    double len = path_dist_ * cell;
    double ex = vx + len * std::cos(rad);
    double ey = vy + len * std::sin(rad);

    p.setPen(QPen(QColor(255, 60, 60), std::max(cell*0.25, 1.5)));
    p.drawLine(QPointF(vx, vy), QPointF(ex, ey));
    double al = std::min(cell*1.2, len*0.3);
    double a1 = rad + M_PI*0.85, a2 = rad - M_PI*0.85;
    p.drawLine(QPointF(ex,ey), QPointF(ex+al*std::cos(a1), ey+al*std::sin(a1)));
    p.drawLine(QPointF(ex,ey), QPointF(ex+al*std::cos(a2), ey+al*std::sin(a2)));
}

// ===================== 车辆 (绿色三角) =====================
void GridMapWidget::drawVehicle(QPainter& p, double ox, double oy, double cell) {
    if (veh_row_ < 0 || veh_col_ < 0) return;
    double vx = ox + veh_col_ * cell + cell/2;
    double vy = oy + (map_rows_ - 1 - veh_row_) * cell + cell/2;  // 翻转
    double sz = std::max(cell * 0.7, 5.0);
    double heading = (direction_ >= 0) ? -M_PI/2 : M_PI/2;

    QPointF tri[3];
    tri[0] = QPointF(vx + sz*std::cos(heading), vy + sz*std::sin(heading));
    tri[1] = QPointF(vx + sz*0.6*std::cos(heading+2.4), vy + sz*0.6*std::sin(heading+2.4));
    tri[2] = QPointF(vx + sz*0.6*std::cos(heading-2.4), vy + sz*0.6*std::sin(heading-2.4));

    p.setPen(QPen(Qt::black, 0.8));
    p.setBrush(QColor(0, 200, 0, 220));
    p.drawPolygon(tri, 3);

    // 坐标标注 (仅格子够大时)
    if (cell >= 6) {
        p.setPen(Qt::white);
        p.setFont(QFont("", 7, QFont::Bold));
        p.drawText(QPointF(vx + sz, vy - sz),
                   QString("(%1,%2)").arg(static_cast<int>(veh_row_)).arg(static_cast<int>(veh_col_)));
    }
}

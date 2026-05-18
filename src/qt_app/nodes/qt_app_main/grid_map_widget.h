/**
 * @file grid_map_widget.h
 * @brief 栅格地图可视化控件 (三并排: 原始地图 / 规划路径 / 实时轨迹)
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 订阅:
 *   /occupancy_grid           — nav_msgs::OccupancyGrid
 *   /Navigate_location        — geometry_msgs::Point (车辆行列)
 *   /Path                     — geometry_msgs::Point (当前路径段)
 *   /Direction                — std_msgs::Float64    (1前进, -1后退)
 *   /decision/full_path       — Float64MultiArray    (完整规划路径)
 *   /decision/waypoint_index  — Float64              (当前路径点索引)
 */
#ifndef GRID_MAP_WIDGET_H
#define GRID_MAP_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMutex>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

class GridMapWidget : public QWidget {
    Q_OBJECT
public:
    explicit GridMapWidget(ros::NodeHandle& nh, QWidget* parent = nullptr);
    QSize sizeHint() const override { return QSize(900, 350); }
    QSize minimumSizeHint() const override { return QSize(450, 200); }
protected:
    void paintEvent(QPaintEvent* event) override;
private:
    // ROS 回调
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void locationCallback(const geometry_msgs::Point::ConstPtr& msg);
    void pathCallback(const geometry_msgs::Point::ConstPtr& msg);
    void directionCallback(const std_msgs::Float64::ConstPtr& msg);
    void fullPathCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // 单张地图绘制 (在指定区域内画)
    void drawMapCells(QPainter& p, double ox, double oy, double cell);
    void drawGrid(QPainter& p, double ox, double oy, double cell);
    void drawPlannedPath(QPainter& p, double ox, double oy, double cell);
    void drawTrail(QPainter& p, double ox, double oy, double cell);
    void drawVehicle(QPainter& p, double ox, double oy, double cell);
    void drawCurrentSegment(QPainter& p, double ox, double oy, double cell);

    ros::Subscriber sub_map_, sub_loc_, sub_path_, sub_dir_;
    ros::Subscriber sub_full_path_, sub_wp_index_;
    QMutex mutex_;

    // 地图数据
    std::vector<int8_t> map_data_;
    int map_rows_ = 0, map_cols_ = 0;
    double resolution_ = 1.0;
    double map_origin_x_ = 0, map_origin_y_ = 0;  ///< 栅格(0,0)的ENU坐标

    // 车辆
    double veh_row_ = -1, veh_col_ = -1;
    double path_theta_ = 0, path_dist_ = 0;
    double direction_ = 1;

    // 规划路径
    struct PathPt { double x, y, dir; };
    std::vector<PathPt> planned_path_;
    int current_wp_index_ = 0;

    // 实时轨迹
    std::vector<std::pair<double,double>> trail_;
    static constexpr int kMaxTrail = 10000;
};

#endif

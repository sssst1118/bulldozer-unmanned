/**
 * @file path_types.h
 * @brief 路径规划与执行的共享数据结构
 * @author dozer-dev
 * @date 2026-03-15
 *
 * @par 铲刀高度控制模型:
 *   铲刀本质上只有一种控制方式 —— 跟踪目标高度 target_height。
 *   BladeCmd 只是"给路径生成器的语义标签", 决定目标高度取什么值:
 *     RAISE → target_height = RAISE_HEIGHT (提刀到最高)
 *     LEVEL → target_height = 由 LevelMode 和对应参数算出 (找平作业)
 *     HOLD  → target_height = 保持上一帧 (路径生成时不改写)
 *
 *   执行器 / moldboard_controller 只看 target_height, 不关心 LevelMode。
 *   LevelMode 只在路径生成阶段起作用。
 */
#ifndef PATH_TYPES_H
#define PATH_TYPES_H

#include <vector>
#include <cmath>

//==============================================================================
// 铲刀指令 (给路径生成器的语义标签)
//==============================================================================
enum class BladeCmd {
    RAISE = 0,  ///< 提刀 (目标高度 = RAISE_HEIGHT, 液压压到最高)
    LEVEL = 1,  ///< 落刀找平 (目标高度 = 按 LevelMode 算出)
    HOLD  = 2   ///< 保持 (路径生成时不改写目标高度)
};

//==============================================================================
// 行驶方向
//==============================================================================
enum class DriveDir {
    FORWARD  =  1,  ///< 前进
    BACKWARD = -1   ///< 后退
};

//==============================================================================
// 路径工艺类型
//==============================================================================
enum class PathMode {
    ZIGZAG              = 0,  ///< 顶端斜移: 推→从末端斜着到下一列起始端→推 (无倒车)
    UNIDIRECTIONAL      = 1,  ///< 底端前进斜移: 推→倒车回底端→斜着前进到下一列→短倒回底端→推
    UNIDI_BACK_SHIFT    = 2   ///< 底端倒车斜移: 推→倒车回底端→斜着倒车到下一列→推 (换列一步完成, 需底端后方有空间)
};

//==============================================================================
// 铲刀找平模式 (决定 BladeCmd::LEVEL 点的 target_height 怎么算)
// 所有模式均以 ENU-Up 绝对高程为目标 (3D找平本质)。
// 不同模式的区别在于"目标高程随路径位置怎么变化"。
//==============================================================================
enum class LevelMode {
    FLAT_3D  = 0,  ///< 找平 + 横坡: 全路径 LEVEL 点推到 target_level_height, 横向跟踪 blade_angle_deg
    SLOPE_3D = 1   ///< 纵坡 + 横坡: 目标高程 = slope_start_height + 行驶距离 × slope_gradient/100, 横向跟踪 blade_angle_deg
};

//==============================================================================
// 常量
//==============================================================================

/**
 * @brief 提刀时的目标高度 (米, ENU-Up 绝对高程)
 *
 * 路径生成器给 BladeCmd::RAISE 的点填这个值。
 * 设得比实际作业高度明显更高, 确保液压把铲刀压到顶 (上限由硬件机械行程决定)。
 * 实际最终高度由 moldboard_controller 内部的 PointD/PointM 阈值限制。
 */
constexpr double RAISE_HEIGHT = 2.0;

//==============================================================================
// 路径点
//==============================================================================
struct WayPoint {
    double   x;             ///< 目标位置 ENU X (米, 相对于作业起点)
    double   y;             ///< 目标位置 ENU Y (米, 相对于作业起点)
    BladeCmd blade_cmd;     ///< 铲刀指令 (语义标签, 执行阶段不依赖它)
    DriveDir drive_dir;     ///< 行驶方向
    double   target_height; ///< 目标铲刀高度 (米, ENU-Up 绝对高程)
                            ///< 由路径生成器按 BladeCmd 和 LevelMode 算出;
                            ///< 执行器下发给 moldboard_controller 作为跟踪目标。

    WayPoint()
        : x(0), y(0),
          blade_cmd(BladeCmd::HOLD),
          drive_dir(DriveDir::FORWARD),
          target_height(0.0) {}

    WayPoint(double _x, double _y, BladeCmd _b, DriveDir _d,
             double _target_height = 0.0)
        : x(_x), y(_y),
          blade_cmd(_b),
          drive_dir(_d),
          target_height(_target_height) {}
};

//==============================================================================
// 路径生成参数
//==============================================================================
struct PathGenParams {
    // ---- 几何与运动参数 (原有) ----
    double blade_width    = 4.2;   ///< 列间距 (米, 由角度和push_length自动计算)
    double push_length    = 20.0;  ///< 单趟推土长度 (米, 沿推土方向)
    double shift_angle    = 12.0;  ///< 斜移角度 (度, 换列对角线与推土方向的夹角)
    double map_width      = 20.0;  ///< 地图宽度 (米, 垂直推土方向), 有地图时用
    double v_push         = 0.5;   ///< 推土速度 (m/s)
    double v_reverse      = 0.8;   ///< 后退速度 (m/s)
    double v_shift        = 0.5;   ///< 换列速度 (m/s)
    double omega_rotate   = 10.0;  ///< 旋转角速度 (deg/s)
    double start_x        = 0.0;   ///< 起点 X
    double start_y        = 0.0;   ///< 起点 Y
    double heading        = 0.0;   ///< 推土方向航向 (度, 北偏东)
    double x_back_set     = 3.0;   ///< 过载后退距离 (米)
    int    num_columns    = 5;     ///< 无地图模式: 列数
    int    passes_per_col = 1;     ///< 无地图模式: 每列遍数 (来回算一遍)
    double map_scale      = 1.0;   ///< 有地图模式: 缩放系数 (0.5=缩小到50%, 2.0=放大到200%)

    // ---- 铲刀找平参数 (新增) ----
    /**
     * @brief 铲刀找平模式
     *
     * 决定 BladeCmd::LEVEL 的路径点 target_height 怎么填。
     * 是个多选一的枚举, 不是开关 —— 每条路径必须指定一种找平方式。
     */
    LevelMode level_mode = LevelMode::FLAT_3D;

    /**
     * @brief FLAT_3D 模式的目标高程 (米, ENU-Up 绝对高程)
     *
     * level_mode = FLAT_3D 时, 所有 BladeCmd::LEVEL 的点 target_height 都填这个值。
     * SLOPE_3D 模式下忽略。
     */
    double target_level_height = 0.0;

    // ---- SLOPE_3D (纵坡 + 横坡) 参数 ----

    /**
     * @brief SLOPE_3D 模式的起始高度 (米, ENU-Up 绝对高程)
     * 推土段起点处的铲刀目标高度。
     */
    double slope_start_height = 0.0;

    /**
     * @brief SLOPE_3D 模式的纵向坡度 (%, 正值上坡负值下坡)
     * 例如 -2.0 表示每行驶 1m 目标高度降低 0.02m (20mm)。
     */
    double slope_gradient = 0.0;

    /// 根据地图宽度和刀宽自动计算列数 (有地图时用)
    int calcNumPasses() const {
        return static_cast<int>(std::ceil(map_width / blade_width));
    }
};

//==============================================================================
// 通用执行器状态
//==============================================================================
enum class ExecState {
    IDLE          = 0,  ///< 空闲, 等待路径
    ROTATING      = 1,  ///< 旋转对准下一个路径点方向
    DRIVING       = 2,  ///< 直行到下一个路径点
    WAYPOINT_DONE = 3,  ///< 到位, 执行铲刀指令, 准备取下一个点
    FINISHED      = 4,  ///< 所有路径点执行完毕
    EMERGENCY     = 5,  ///< 异常停车 (RTK丢失/过载等)
    OVERLOAD_BACK = 6   ///< 过载处理: 提刀后退中
};

#endif // PATH_TYPES_H

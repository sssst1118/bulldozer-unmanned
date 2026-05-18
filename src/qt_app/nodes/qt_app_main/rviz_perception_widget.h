/**
 * @file rviz_perception_widget.h
 * @brief 嵌入式 RViz 感知视图面板 (带完整容错)
 * @date 2026-04
 *
 * 安全策略:
 *   1. 运行时: try-catch 包裹所有 rviz 调用, 初始化失败降级为提示页面
 *   2. 数据为空: 各 Display 创建失败时跳过, 不影响其他图层
 *   3. 话题无数据: RViz 原生行为 (显示空场景 + Grid), 不崩溃
 *   4. rviz 未安装: 动态检测, 显示安装指引
 */
#ifndef RVIZ_PERCEPTION_WIDGET_H
#define RVIZ_PERCEPTION_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QComboBox>

// 前向声明
namespace rviz {
    class RenderPanel;
    class VisualizationManager;
    class Display;
}

class RvizPerceptionWidget : public QWidget {
    Q_OBJECT
public:
    explicit RvizPerceptionWidget(QWidget* parent = nullptr);
    ~RvizPerceptionWidget() override;

    /// RViz 是否成功初始化
    bool isRvizReady() const { return rviz_ready_; }

    /// 确保 RViz 已初始化 (首次切到感知视图时调用, 延迟初始化以加快启动)
    /// 返回值: true=已就绪(含初始化成功或之前已初始化), false=不可用/失败
    bool ensureInitialized();

    /// 暂停 RViz 渲染 + 禁用所有 Display (切到其他视图时调用, 彻底断订阅)
    void pauseRendering();
    /// 恢复 RViz 渲染 + 按 checkbox 状态恢复 Display (切回感知视图时调用)
    void resumeRendering();

private:
    bool checkRvizAvailable();
    void initRviz();
    void setupToolbar();
    void showFallbackPage(const QString& title, const QString& detail);

    /// 暂停时: 停渲染 + 禁用所有 Display 让其取消话题订阅
    void suspendAll();
    /// 恢复时: 开渲染 + 按 checkbox 勾选状态重新启用 Display
    void restoreAll();

    /// 安全创建 Display, 失败返回 nullptr
    rviz::Display* safeCreateDisplay(const QString& classId,
                                     const QString& name, bool enabled);
    /// 安全设置属性
    void safeSetProp(rviz::Display* disp, const QString& prop, const QVariant& val);

    void toggleDisplay(rviz::Display* display, bool on);

    void setTopDownView();
    void setFollowView();
    void set3DView();

    // 布局
    QVBoxLayout* main_layout_ = nullptr;
    QWidget* toolbar_ = nullptr;

    // RViz 核心
    rviz::RenderPanel* render_panel_ = nullptr;
    rviz::VisualizationManager* manager_ = nullptr;
    bool rviz_ready_ = false;
    /// 是否已尝试过初始化 (避免重复进入 initRviz)
    bool init_attempted_ = false;
    /// 当前是否处于渲染暂停状态 (ensureInitialized 首次进入时需判断)
    bool is_suspended_ = true;

    // Displays (任何一个都可能为 nullptr)
    rviz::Display* disp_grid_ = nullptr;
    rviz::Display* disp_tf_ = nullptr;
    rviz::Display* disp_merged_cloud_ = nullptr;
    rviz::Display* disp_allmap_ = nullptr;
    rviz::Display* disp_front_rgba_ = nullptr;
    rviz::Display* disp_behind_rgba_ = nullptr;
    rviz::Display* disp_rtk_path_ = nullptr;
    rviz::Display* disp_front_img_ = nullptr;
    rviz::Display* disp_back_img_ = nullptr;
    rviz::Display* disp_left_img_ = nullptr;
    rviz::Display* disp_right_img_ = nullptr;

    // 工具栏控件
    QCheckBox* chk_merged_cloud_ = nullptr;
    QCheckBox* chk_allmap_ = nullptr;
    QCheckBox* chk_front_rgba_ = nullptr;
    QCheckBox* chk_behind_rgba_ = nullptr;
    QCheckBox* chk_rtk_path_ = nullptr;
    QCheckBox* chk_front_img_ = nullptr;
    QCheckBox* chk_back_img_ = nullptr;
    QCheckBox* chk_left_img_ = nullptr;
    QCheckBox* chk_right_img_ = nullptr;
    QComboBox* combo_view_preset_ = nullptr;
};

#endif

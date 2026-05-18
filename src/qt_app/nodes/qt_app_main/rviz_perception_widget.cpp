/**
 * @file rviz_perception_widget.cpp
 * @brief 嵌入式 RViz 感知视图面板 — 带完整容错
 * @date 2026-04
 *
 * 所有 rviz 调用均在 try-catch 内, 任何异常/崩溃都降级为文字提示页,
 * 不会导致主程序退出。
 */
#include "rviz_perception_widget.h"

#include <QProcess>
#include <QFile>
#include <QFont>
#include <ros/ros.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/tool_manager.h>

//==============================================================================
// 构造 / 析构
//==============================================================================
RvizPerceptionWidget::RvizPerceptionWidget(QWidget* parent)
    : QWidget(parent)
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setSpacing(0);
    main_layout_->setContentsMargins(0, 0, 0, 0);

    // 延迟初始化策略:
    //   - 构造阶段只建工具栏 (轻量, 不触发 OGRE/话题订阅)
    //   - 真正的 rviz 初始化推迟到首次切入"感知视图"时由 ensureInitialized() 触发
    //   - 可显著缩短主窗口启动时间 (1-2 秒)
    setupToolbar();
}

RvizPerceptionWidget::~RvizPerceptionWidget()
{
    try {
        if (manager_) {
            manager_->stopUpdate();
            delete manager_;
            manager_ = nullptr;
        }
    } catch (...) {
        // 析构中不抛异常
        manager_ = nullptr;
    }
}

//==============================================================================
// 检测 rviz 是否可用
//==============================================================================
bool RvizPerceptionWidget::checkRvizAvailable()
{
    // 方法1: 检查 librviz.so 是否能被加载 (编译时已链接, 运行时检查)
    // 方法2: 检查 rospack find rviz 是否成功
    QProcess proc;
    proc.start("rospack", QStringList() << "find" << "rviz");
    proc.waitForFinished(3000);
    if (proc.exitCode() != 0) {
        ROS_WARN("[RvizPerception] rospack find rviz failed, rviz not installed");
        return false;
    }
    return true;
}

//==============================================================================
// 降级提示页面 (rviz 不可用 或 初始化失败时显示)
//==============================================================================
void RvizPerceptionWidget::showFallbackPage(const QString& title, const QString& detail)
{
    // 隐藏工具栏
    if (toolbar_) toolbar_->setVisible(false);

    auto* page = new QWidget(this);
    page->setStyleSheet("background: #F9FAFB;");
    auto* vl = new QVBoxLayout(page);
    vl->setAlignment(Qt::AlignCenter);
    vl->setSpacing(16);

    // 图标
    auto* icon = new QLabel("\u26A0", page);
    icon->setFont(QFont("", 48));
    icon->setAlignment(Qt::AlignCenter);
    icon->setStyleSheet("color: #FF9500;");
    vl->addWidget(icon);

    // 标题
    auto* titleLabel = new QLabel(title, page);
    titleLabel->setFont(QFont("", 16, QFont::Bold));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("color: #000000;");
    vl->addWidget(titleLabel);

    // 详细说明
    auto* detailLabel = new QLabel(detail, page);
    detailLabel->setFont(QFont("Monospace", 11));
    detailLabel->setAlignment(Qt::AlignCenter);
    detailLabel->setWordWrap(true);
    detailLabel->setStyleSheet("color: #8E8E93; padding: 10px 40px;"
                               "background: white; border-radius: 12px;"
                               "border: 1px solid #E0E0E5;");
    detailLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    vl->addWidget(detailLabel);

    main_layout_->addWidget(page, 1);
}

//==============================================================================
// 工具栏
//==============================================================================
void RvizPerceptionWidget::setupToolbar()
{
    toolbar_ = new QWidget(this);
    toolbar_->setFixedHeight(36);
    toolbar_->setStyleSheet(
        "QWidget{background:#E0E0E5;border-bottom:1px solid #D1D1D6;}"
        "QCheckBox{color:#000000;font-size:11px;padding:2px 6px;}"
        "QCheckBox::indicator{width:14px;height:14px;}"
        "QPushButton{background:#F9FAFB;color:#007AFF;border:1px solid #D1D1D6;"
        "  border-radius:4px;padding:3px 10px;font-size:11px;font-weight:600;}"
        "QPushButton:hover{background:#D1D1D6;}"
        "QComboBox{background:#F9FAFB;color:#000000;border:1px solid #D1D1D6;"
        "  border-radius:4px;padding:2px 8px;font-size:11px;}"
        "QLabel{color:#8E8E93;font-size:11px;padding:0 4px;}");

    auto* hl = new QHBoxLayout(toolbar_);
    hl->setContentsMargins(8, 2, 8, 2);
    hl->setSpacing(6);

    auto* layerLabel = new QLabel("\u56FE\u5C42:", toolbar_);
    hl->addWidget(layerLabel);

    chk_merged_cloud_ = new QCheckBox("\u5408\u5E76\u70B9\u4E91", toolbar_);
    chk_merged_cloud_->setChecked(true);
    hl->addWidget(chk_merged_cloud_);

    chk_allmap_ = new QCheckBox("\u5168\u5C40\u5730\u56FE", toolbar_);
    chk_allmap_->setChecked(false);
    hl->addWidget(chk_allmap_);

    chk_front_rgba_ = new QCheckBox("\u524D\u96F7\u8FBE\u8BED\u4E49", toolbar_);
    chk_front_rgba_->setChecked(true);
    hl->addWidget(chk_front_rgba_);

    chk_behind_rgba_ = new QCheckBox("\u540E\u96F7\u8FBE\u8BED\u4E49", toolbar_);
    chk_behind_rgba_->setChecked(true);
    hl->addWidget(chk_behind_rgba_);

    chk_rtk_path_ = new QCheckBox("RTK\u8F68\u8FF9", toolbar_);
    chk_rtk_path_->setChecked(true);
    hl->addWidget(chk_rtk_path_);

    chk_front_img_ = new QCheckBox("\u524D\u56FE\u50CF", toolbar_);
    chk_front_img_->setChecked(false);
    hl->addWidget(chk_front_img_);

    chk_back_img_ = new QCheckBox("\u540E\u56FE\u50CF", toolbar_);
    chk_back_img_->setChecked(false);
    hl->addWidget(chk_back_img_);

    chk_left_img_ = new QCheckBox("\u5DE6\u56FE\u50CF", toolbar_);
    chk_left_img_->setChecked(false);
    hl->addWidget(chk_left_img_);

    chk_right_img_ = new QCheckBox("\u53F3\u56FE\u50CF", toolbar_);
    chk_right_img_->setChecked(false);
    hl->addWidget(chk_right_img_);

    hl->addStretch();

    auto* viewLabel = new QLabel("\u89C6\u89D2:", toolbar_);
    hl->addWidget(viewLabel);

    combo_view_preset_ = new QComboBox(toolbar_);
    combo_view_preset_->addItems({"\u4FEF\u89C6\u56FE", "\u8DDF\u968F\u8F66\u8F86", "3D\u81EA\u7531"});
    hl->addWidget(combo_view_preset_);

    auto* btnReset = new QPushButton("\u91CD\u7F6E\u89C6\u89D2", toolbar_);
    hl->addWidget(btnReset);

    main_layout_->addWidget(toolbar_, 0);

    // 信号连接 (display 指针在 initRviz 后才有效, toggleDisplay 内部会检查 nullptr)
    connect(chk_merged_cloud_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_merged_cloud_, on); });
    connect(chk_allmap_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_allmap_, on); });
    connect(chk_front_rgba_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_front_rgba_, on); });
    connect(chk_behind_rgba_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_behind_rgba_, on); });
    connect(chk_rtk_path_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_rtk_path_, on); });
    connect(chk_front_img_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_front_img_, on); });
    connect(chk_back_img_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_back_img_, on); });
    connect(chk_left_img_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_left_img_, on); });
    connect(chk_right_img_, &QCheckBox::toggled, [this](bool on){ toggleDisplay(disp_right_img_, on); });

    connect(combo_view_preset_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx){
        if (!rviz_ready_) return;
        switch(idx) {
            case 0: setTopDownView(); break;
            case 1: setFollowView(); break;
            case 2: set3DView(); break;
        }
    });

    connect(btnReset, &QPushButton::clicked, [this]{
        if (!rviz_ready_) return;
        combo_view_preset_->setCurrentIndex(0);
        setTopDownView();
    });
}

//==============================================================================
// 安全创建 Display
//==============================================================================
rviz::Display* RvizPerceptionWidget::safeCreateDisplay(
    const QString& classId, const QString& name, bool enabled)
{
    if (!manager_) return nullptr;
    try {
        auto* disp = manager_->createDisplay(classId, name, enabled);
        if (!disp) {
            ROS_WARN("[RvizPerception] Failed to create display: %s (%s)",
                     name.toStdString().c_str(), classId.toStdString().c_str());
        }
        return disp;
    } catch (const std::exception& e) {
        ROS_ERROR("[RvizPerception] Exception creating display %s: %s",
                  name.toStdString().c_str(), e.what());
        return nullptr;
    } catch (...) {
        ROS_ERROR("[RvizPerception] Unknown exception creating display %s",
                  name.toStdString().c_str());
        return nullptr;
    }
}

//==============================================================================
// 安全设置属性
//==============================================================================
void RvizPerceptionWidget::safeSetProp(rviz::Display* disp,
                                       const QString& prop,
                                       const QVariant& val)
{
    if (!disp) return;
    try {
        auto* p = disp->subProp(prop);
        if (p) p->setValue(val);
    } catch (...) {
        ROS_WARN("[RvizPerception] Failed to set property %s",
                 prop.toStdString().c_str());
    }
}

//==============================================================================
// 初始化 librviz (全部 try-catch 保护)
//==============================================================================
void RvizPerceptionWidget::initRviz()
{
    try {
        render_panel_ = new rviz::RenderPanel(this);
        manager_ = new rviz::VisualizationManager(render_panel_);
        render_panel_->initialize(manager_->getSceneManager(), manager_);
        manager_->initialize();
        manager_->startUpdate();
        manager_->setFixedFrame("base_link");
    } catch (const std::exception& e) {
        ROS_ERROR("[RvizPerception] RViz init failed: %s", e.what());
        showFallbackPage(
            "RViz \u521D\u59CB\u5316\u5931\u8D25",
            QString("\u9519\u8BEF\u4FE1\u606F: %1\n\n"
                    "\u53EF\u80FD\u539F\u56E0:\n"
                    "  - \u663E\u5361\u9A71\u52A8\u672A\u5B89\u88C5\u6216\u4E0D\u517C\u5BB9\n"
                    "  - OpenGL \u4E0D\u53EF\u7528 (\u5C1D\u8BD5: export LIBGL_ALWAYS_SOFTWARE=1)\n"
                    "  - DISPLAY \u73AF\u5883\u53D8\u91CF\u672A\u8BBE\u7F6E\n\n"
                    "\u8BF7\u68C0\u67E5\u4EE5\u4E0A\u95EE\u9898\u540E\u91CD\u542F\u7A0B\u5E8F\u3002").arg(e.what()));
        return;
    } catch (...) {
        ROS_ERROR("[RvizPerception] RViz init failed with unknown exception");
        showFallbackPage(
            "RViz \u521D\u59CB\u5316\u5931\u8D25",
            "\u53D1\u751F\u672A\u77E5\u9519\u8BEF\u3002\n\n"
            "\u5C1D\u8BD5:\n"
            "  1. sudo apt install ros-noetic-rviz\n"
            "  2. export LIBGL_ALWAYS_SOFTWARE=1\n"
            "  3. \u91CD\u542F\u7A0B\u5E8F");
        return;
    }

    // ── 创建 Displays (每个独立 try-catch, 失败不影响其他) ──

    // Grid
    disp_grid_ = safeCreateDisplay("rviz/Grid", "Grid", true);
    safeSetProp(disp_grid_, "Plane Cell Count", 40);
    safeSetProp(disp_grid_, "Cell Size", 1.0);
    safeSetProp(disp_grid_, "Color", QColor(200, 200, 200, 60));

    // TF
    disp_tf_ = safeCreateDisplay("rviz/TF", "TF", true);
    safeSetProp(disp_tf_, "Show Names", true);
    safeSetProp(disp_tf_, "Show Axes", true);

    // 合并点云
    disp_merged_cloud_ = safeCreateDisplay("rviz/PointCloud2",
        "\u5408\u5E76\u70B9\u4E91", true);
    safeSetProp(disp_merged_cloud_, "Topic", "/hesai_lidar_base_link");
    safeSetProp(disp_merged_cloud_, "Size (m)", 0.05);
    safeSetProp(disp_merged_cloud_, "Style", "Points");
    safeSetProp(disp_merged_cloud_, "Color Transformer", "Intensity");

    // 全局地图 (默认关)
    disp_allmap_ = safeCreateDisplay("rviz/PointCloud2",
        "\u5168\u5C40\u5730\u56FE", false);
    safeSetProp(disp_allmap_, "Topic", "/Map/AllMap");
    safeSetProp(disp_allmap_, "Size (m)", 0.1);
    safeSetProp(disp_allmap_, "Style", "Points");
    safeSetProp(disp_allmap_, "Color Transformer", "AxisColor");
    safeSetProp(disp_allmap_, "Decay Time", 0);

    // 前雷达语义
    disp_front_rgba_ = safeCreateDisplay("rviz/PointCloud2",
        "\u524D\u96F7\u8FBE\u8BED\u4E49", true);
    safeSetProp(disp_front_rgba_, "Topic", "/front_lidar_points_rgba");
    safeSetProp(disp_front_rgba_, "Size (m)", 0.08);
    safeSetProp(disp_front_rgba_, "Style", "Points");
    safeSetProp(disp_front_rgba_, "Color Transformer", "RGB8");

    // 后雷达语义
    disp_behind_rgba_ = safeCreateDisplay("rviz/PointCloud2",
        "\u540E\u96F7\u8FBE\u8BED\u4E49", true);
    safeSetProp(disp_behind_rgba_, "Topic", "/behind_lidar_points_rgba");
    safeSetProp(disp_behind_rgba_, "Size (m)", 0.08);
    safeSetProp(disp_behind_rgba_, "Style", "Points");
    safeSetProp(disp_behind_rgba_, "Color Transformer", "RGB8");

    // RTK 轨迹
    disp_rtk_path_ = safeCreateDisplay("rviz/Path",
        "RTK\u8F68\u8FF9", true);
    safeSetProp(disp_rtk_path_, "Topic", "rtk_path");
    safeSetProp(disp_rtk_path_, "Color", QColor(0, 200, 80));
    safeSetProp(disp_rtk_path_, "Line Style", "Lines");

    // 前方检测图像 (默认关)
    disp_front_img_ = safeCreateDisplay("rviz/Image",
        "\u524D\u65B9\u56FE\u50CF", false);
    if (disp_front_img_) {
        safeSetProp(disp_front_img_, "Image Topic", "/front_img_res");
    }

    // 后方检测图像 (默认关)
    disp_back_img_ = safeCreateDisplay("rviz/Image",
        "\u540E\u65B9\u56FE\u50CF", false);
    if (disp_back_img_) {
        safeSetProp(disp_back_img_, "Image Topic", "/back_img_res");
    }

    // 左侧检测图像 (默认关)
    disp_left_img_ = safeCreateDisplay("rviz/Image",
        "\u5DE6\u4FA7\u56FE\u50CF", false);
    if (disp_left_img_) {
        safeSetProp(disp_left_img_, "Image Topic", "/left_img_res");
    }

    // 右侧检测图像 (默认关)
    disp_right_img_ = safeCreateDisplay("rviz/Image",
        "\u53F3\u4FA7\u56FE\u50CF", false);
    if (disp_right_img_) {
        safeSetProp(disp_right_img_, "Image Topic", "/right_img_res");
    }

    // 加入布局
    main_layout_->addWidget(render_panel_, 1);
    rviz_ready_ = true;

    // 默认俯视
    setTopDownView();

    ROS_INFO("[RvizPerception] RViz initialized successfully");
}

//==============================================================================
// 图层开关 (安全)
//==============================================================================
void RvizPerceptionWidget::toggleDisplay(rviz::Display* display, bool on)
{
    if (!display || !rviz_ready_) return;
    // 暂停态下用户勾选 checkbox, 仅更新 UI 状态, 不真正启用 Display
    // (避免在后台订阅话题堆积数据); restoreAll 时会按 checkbox 状态同步
    if (is_suspended_) return;
    try {
        display->setEnabled(on);
    } catch (...) {
        ROS_WARN("[RvizPerception] Failed to toggle display");
    }
}

//==============================================================================
// 视角预设 (全部安全)
//==============================================================================
void RvizPerceptionWidget::setTopDownView()
{
    if (!manager_ || !rviz_ready_) return;
    try {
        manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
        auto* vc = manager_->getViewManager()->getCurrent();
        if (vc) {
            vc->subProp("Scale")->setValue(30);
            vc->subProp("X")->setValue(0);
            vc->subProp("Y")->setValue(0);
            vc->subProp("Angle")->setValue(0);
        }
    } catch (...) {
        ROS_WARN("[RvizPerception] Failed to set top-down view");
    }
}

void RvizPerceptionWidget::setFollowView()
{
    if (!manager_ || !rviz_ready_) return;
    try {
        manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        auto* vc = manager_->getViewManager()->getCurrent();
        if (vc) {
            vc->subProp("Distance")->setValue(30);
            vc->subProp("Pitch")->setValue(0.8);
            vc->subProp("Yaw")->setValue(3.14);
            vc->subProp("Focal Point")->subProp("X")->setValue(0);
            vc->subProp("Focal Point")->subProp("Y")->setValue(0);
            vc->subProp("Focal Point")->subProp("Z")->setValue(0);
        }
        manager_->setFixedFrame("base_link");
    } catch (...) {
        ROS_WARN("[RvizPerception] Failed to set follow view");
    }
}

void RvizPerceptionWidget::set3DView()
{
    if (!manager_ || !rviz_ready_) return;
    try {
        manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        auto* vc = manager_->getViewManager()->getCurrent();
        if (vc) {
            vc->subProp("Distance")->setValue(50);
            vc->subProp("Pitch")->setValue(0.5);
            vc->subProp("Yaw")->setValue(-1.57);
            vc->subProp("Focal Point")->subProp("X")->setValue(0);
            vc->subProp("Focal Point")->subProp("Y")->setValue(0);
            vc->subProp("Focal Point")->subProp("Z")->setValue(0);
        }
    } catch (...) {
        ROS_WARN("[RvizPerception] Failed to set 3D view");
    }
}

//==============================================================================
// 延迟初始化入口 (首次切到感知视图时调用)
//==============================================================================
bool RvizPerceptionWidget::ensureInitialized()
{
    // 已初始化过 (无论成功失败), 不重复进入
    if (init_attempted_) {
        return rviz_ready_;
    }
    init_attempted_ = true;

    // 第一步: 检测 rviz 是否可用
    if (!checkRvizAvailable()) {
        showFallbackPage(
            "\u672A\u68C0\u6D4B\u5230 RViz",
            "\u8BF7\u5B89\u88C5 RViz \u4EE5\u4F7F\u7528\u611F\u77E5\u89C6\u56FE\u529F\u80FD\u3002\n\n"
            "\u5B89\u88C5\u547D\u4EE4:\n"
            "  sudo apt update\n"
            "  sudo apt install ros-noetic-rviz\n\n"
            "\u5B89\u88C5\u5B8C\u6210\u540E\u91CD\u542F\u7A0B\u5E8F\u5373\u53EF\u3002");
        return false;
    }

    // 第二步: 真正初始化 rviz (带异常保护)
    initRviz();

    // initRviz 内部的 startUpdate 已让渲染运行, 标记非暂停
    if (rviz_ready_) {
        is_suspended_ = false;
    }
    return rviz_ready_;
}

//==============================================================================
// 暂停/恢复 (对外接口)
// 切走时: 停渲染 + 禁用所有 Display (彻底断话题订阅, 避免后台堆积卡死)
// 切回时: 开渲染 + 按 checkbox 状态恢复 Display 启用状态
//==============================================================================
void RvizPerceptionWidget::pauseRendering()
{
    // 若从未初始化过, 无需暂停 (还没启动)
    if (!init_attempted_ || !rviz_ready_) return;
    if (is_suspended_) return;
    suspendAll();
    is_suspended_ = true;
}

void RvizPerceptionWidget::resumeRendering()
{
    if (!init_attempted_ || !rviz_ready_) return;
    if (!is_suspended_) return;
    restoreAll();
    is_suspended_ = false;
}

//==============================================================================
// 暂停所有 Display (禁用即取消话题订阅, RViz 原生行为)
//==============================================================================
void RvizPerceptionWidget::suspendAll()
{
    if (!manager_) return;
    try {
        manager_->stopUpdate();
    } catch (...) {}

    // Grid/TF 是本地 Display, 禁用开销低无所谓, 但保持一致禁用
    rviz::Display* all[] = {
        disp_grid_, disp_tf_,
        disp_merged_cloud_, disp_allmap_,
        disp_front_rgba_, disp_behind_rgba_, disp_rtk_path_,
        disp_front_img_, disp_back_img_, disp_left_img_, disp_right_img_
    };
    for (auto* d : all) {
        if (!d) continue;
        try { d->setEnabled(false); } catch (...) {}
    }
}

//==============================================================================
// 恢复所有 Display (按 checkbox 当前勾选状态)
//==============================================================================
void RvizPerceptionWidget::restoreAll()
{
    if (!manager_) return;

    // Grid/TF 始终开
    try { if (disp_grid_) disp_grid_->setEnabled(true); } catch (...) {}
    try { if (disp_tf_) disp_tf_->setEnabled(true); } catch (...) {}

    // 其余按 checkbox
    auto restore = [](rviz::Display* d, QCheckBox* cb) {
        if (!d || !cb) return;
        try { d->setEnabled(cb->isChecked()); } catch (...) {}
    };
    restore(disp_merged_cloud_, chk_merged_cloud_);
    restore(disp_allmap_,       chk_allmap_);
    restore(disp_front_rgba_,   chk_front_rgba_);
    restore(disp_behind_rgba_,  chk_behind_rgba_);
    restore(disp_rtk_path_,     chk_rtk_path_);
    restore(disp_front_img_,    chk_front_img_);
    restore(disp_back_img_,     chk_back_img_);
    restore(disp_left_img_,     chk_left_img_);
    restore(disp_right_img_,    chk_right_img_);

    try {
        manager_->startUpdate();
    } catch (...) {}
}

/**
 * @file grouped_section.h
 * @brief iOS 分组卡片 (融合风补充篇·补充一)
 * @author dozer-dev
 * @date 2026-05-07
 *
 * iOS 风分组列表的视觉容器: 大写灰色组标题 + 圆角白卡片 + 行间极浅分割.
 * 配合 ParameterRow (一行=一个参数) 使用, 可把零散 QGridLayout 重排成
 * iOS Settings 风格的"分组列表".
 *
 * 视觉构成 (paintEvent):
 *   - 标题: widget 上方 margin 区, 大写, labelFont(11), Theme::textSecondary
 *   - 卡片: ShadowUtils::drawCardShadow + Squircle 14px 圆角 + 白底
 *   - 顶部 1px 高光线 (alpha 60)
 *   - 行间 0.5px 极浅分割线 (左侧缩进 20px, 仅在多行时绘制)
 *
 * 用法:
 *   auto* sec = new GroupedSection("路径模式", parent);
 *   sec->addRow(new ParameterRow("斜移角度", spin_shift_, sec));
 *   sec->addRow(new ParameterRow("推土方向", spin_heading_, sec));
 *   layout->addWidget(sec);
 *   layout->addSpacing(Spacing::sectionGap);  // 35px 组间距
 *
 * 约定: 无 Q_OBJECT, header-only.
 */
#ifndef QT_APP_GROUPED_SECTION_H
#define QT_APP_GROUPED_SECTION_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QString>
#include "smooth_ui.h"
#include "squircle.h"
#include "shadow_utils.h"

class GroupedSection : public QWidget {
public:
    explicit GroupedSection(const QString& header, QWidget* parent = nullptr)
        : QWidget(parent), header_(header)
    {
        // 顶部预留标题区 (header_height + 6 间隙); 卡片本身有 padding 不在这里加.
        layout_ = new QVBoxLayout(this);
        layout_->setSpacing(0);
        layout_->setContentsMargins(0, headerArea(), 0, 0);
    }

    /// 追加一行 (ParameterRow / 任意 QWidget)
    void addRow(QWidget* row) {
        if (!row) return;
        rows_.push_back(row);
        layout_->addWidget(row);
    }

    /// 设置 / 替换标题
    void setHeader(const QString& h) { header_ = h; update(); }
    QString header() const { return header_; }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // ---- 标题 (在 widget 顶部 margin 区, 不在卡片内) ----
        if (!header_.isEmpty()) {
            QFont f = SmoothUI::labelFont(11);
            f.setLetterSpacing(QFont::AbsoluteSpacing, 0.6);
            p.setFont(f);
            p.setPen(Theme::textSecondary);
            QRect titleRect(20, 0, width() - 40, headerArea() - 6);
            p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter,
                       header_.toUpper());
        }

        // ---- 卡片本体: 阴影 + Squircle 白底 ----
        QRect card(0, headerArea(), width(), height() - headerArea());
        if (card.height() <= 0) return;

        ShadowUtils::drawCardShadow(p, card, Spacing::panelRadius);
        Squircle::draw(p, QRectF(card),
                       Spacing::panelRadius,
                       QBrush(Theme::bgSurface));

        // ---- 顶部 1px 高光线 (iOS 卡片上沿反光) ----
        QPainterPath clip = Squircle::path(QRectF(card),
                                           Spacing::panelRadius);
        p.save();
        p.setClipPath(clip);
        QLinearGradient topLight(0, card.top(), 0, card.top() + 1);
        topLight.setColorAt(0.0, QColor(255, 255, 255, 110));
        topLight.setColorAt(1.0, Qt::transparent);
        p.fillRect(card.left(), card.top(), card.width(), 1, topLight);
        p.restore();

        // ---- 行间分割线 (左侧缩进 20px, 0.5px 极浅) ----
        if (rows_.size() > 1) {
            QPen sep(QColor(0, 0, 0, 18));
            sep.setWidthF(0.5);
            p.setPen(sep);
            for (size_t i = 0; i + 1 < rows_.size(); ++i) {
                QWidget* row = rows_[i];
                if (!row || !row->isVisible()) continue;
                int y = row->geometry().bottom();
                p.drawLine(card.left() + 20, y,
                           card.right() - 1, y);
            }
        }
    }

private:
    static constexpr int headerArea() { return 26; }  // 标题文字 + 6px 间距

    QString header_;
    QVBoxLayout* layout_ = nullptr;
    std::vector<QWidget*> rows_;
};

#endif // QT_APP_GROUPED_SECTION_H

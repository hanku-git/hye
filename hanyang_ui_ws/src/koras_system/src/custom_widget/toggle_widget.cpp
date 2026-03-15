/****************************************************************************
**
** This file is part of the Oxygen2 project.
**
** SPDX-FileCopyrightText: 2022 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>
**
** SPDX-License-Identifier: MIT
**
****************************************************************************/

#include "toggle_widget.hpp"
#include "colorrepository.hpp"

#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>

static const int s_height = 26;
static const int s_innerMargin = 4;
static const int s_handleSize = s_height - s_innerMargin * 2;
static const int s_width = s_handleSize * 2 + s_innerMargin * 2;

ToggleWidget::ToggleWidget(QWidget *parent)
    : QWidget{parent}
{
    setSizePolicy({QSizePolicy::Fixed, QSizePolicy::Fixed}); // sizeHint is the only acceptable size
    setFocusPolicy(Qt::TabFocus); // can tab into the widget
    setAttribute(Qt::WA_Hover); // repaint on mouse-enter/mouse-exit
}

void ToggleWidget::setChecked(bool checked)
{
    if (m_checked == checked)
        return;
    m_checked = checked;
    Q_EMIT toggled(checked);
    update();
}

bool ToggleWidget::isChecked() const
{
    return m_checked;
}

void ToggleWidget::toggle()
{
    setChecked(!m_checked);
}

QSize ToggleWidget::sizeHint() const
{
    return QSize(s_width, s_height);
}

void ToggleWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (!isEnabled()) {
        painter.setPen(ColorRepository::buttonOutlineColor());
        painter.setOpacity(0.5);
    } else if (m_mouseDown) // Sunken
        painter.setPen(ColorRepository::pressedOutlineColor());
    else if (underMouse() || hasFocus())
        painter.setPen(QPen(ColorRepository::hoverOutlineBrush(rect()), 1));
    else
        painter.setPen(ColorRepository::buttonOutlineColor());

    if (m_checked)
        painter.setBrush(ColorRepository::baseBackground());
    const qreal radius = height() / 2;
    painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5), radius, radius);

    // Now draw the handle

    QRect valueRect = rect().adjusted(s_innerMargin, s_innerMargin, -s_innerMargin, -s_innerMargin);
    valueRect.setWidth(valueRect.height()); // must be a square

    if (m_checked) {
        valueRect.moveLeft(width() / 2);
        painter.setPen(QPen(ColorRepository::progressBarOutlineBrush(valueRect), 1));
        painter.setBrush(Qt::NoBrush);
    } else {
        painter.setBrush(ColorRepository::baseBackground());
    }
    painter.drawEllipse(valueRect);
}

void ToggleWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_mouseDown = true;
    } else {
        event->ignore();
    }
}

void ToggleWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_mouseDown) {
        m_mouseDown = false;
        toggle();
        Q_EMIT checked(m_checked);
    } else {
        event->ignore();
    }
}

void ToggleWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space) {
        toggle();
        Q_EMIT checked(m_checked);
    } else {
        event->ignore(); // let it propagate to the parent (e.g. so that Return closes dialogs)
    }
}
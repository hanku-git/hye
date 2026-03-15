/****************************************************************************
**
** This file is part of the Oxygen2 project.
**
** SPDX-FileCopyrightText: 2022 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>
**
** SPDX-License-Identifier: MIT
**
****************************************************************************/

#ifndef TOGGLE_WIDGET_HPP
#define TOGGLE_WIDGET_HPP

#include <QtWidgets/QWidget>

class ToggleWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(bool checked READ isChecked WRITE setChecked NOTIFY toggled)
public:
    explicit ToggleWidget(QWidget *parent = nullptr);

    void setChecked(bool checked);
    bool isChecked() const;

    void toggle();

    QSize sizeHint() const override;

Q_SIGNALS:
    void checked(bool checked); 
    void toggled(bool checked); 

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    bool m_checked = false;
    bool m_mouseDown = false;
};

#endif
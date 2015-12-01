/**
 * Copyright (C) 2009-2016  Przemysław Dobrowolski
 *
 * This file is part of the Configuration Space Library (libcs), a library
 * for creating configuration spaces of various motion planning problems.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RENDER_VIEW_FLY_CAMERA_H
#define RENDER_VIEW_FLY_CAMERA_H

#include "renderviewcamera.h"
#include <QVector3D>

class RenderViewFlyCamera
    : public RenderViewCamera
{
private:
    // Left dragging
    bool m_dragLeft;
    QPoint m_dragLeftBeginLocation;
    float m_dragLeftBeginYaw;
    float m_dragLeftBeginPitch;

    // Middle dragging
    bool m_dragMiddle;
    QPoint m_dragMiddleBeginLocation;
    QVector3D m_dragMiddleBeginPosition;

    // Private properties
    float m_yaw;
    float m_pitch;

    // Public properties
    float m_sensitivity;
    float m_moveSensitivity;
    QVector3D m_position;

    QVector3D forwardVector();
    QVector3D strafeRightVector();
    QVector3D sphericalVector(float yaw, float pitch);

public:
    RenderViewFlyCamera();
    virtual ~RenderViewFlyCamera();

    virtual void mouseButtonPressed(const QRect &panelRectangle, Qt::MouseButton buttons, const QPoint &location);
    virtual void mouseMoved(const QRect &panelRectangle, const QPoint &location);
    virtual void mouseButtonReleased(const QRect &panelRectangle, Qt::MouseButton buttons, const QPoint &location);
    virtual void mouseWheelMoved(const QRect &panelRectangle, int wheelDelta);

    virtual void applyTransformGL();
    virtual QVector3D cameraPosition() const;
    virtual double zoom() const;

    virtual void moveForward(double amount);
    virtual void strafeRight(double amount);
};

#endif // RENDER_VIEW_FLY_CAMERA_H

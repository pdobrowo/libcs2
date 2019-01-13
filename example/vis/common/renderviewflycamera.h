/**
 * Copyright (c) 2015-2019 Przemysław Dobrowolski
 *
 * This file is part of the Configuration Space Library (libcs2), a library
 * for creating configuration spaces of various motion planning problems.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

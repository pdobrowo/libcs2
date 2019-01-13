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
#ifndef RENDER_VIEW_CAMERA_H
#define RENDER_VIEW_CAMERA_H

#include <QRect>
#include <QObject>
#include <QVector3D>

class RenderViewCamera
    : public QObject
{
    Q_OBJECT

public:
    virtual ~RenderViewCamera() {}

    // Mouse handling
    virtual void mouseButtonPressed(const QRect &panelRectangle, Qt::MouseButton button, const QPoint &location) = 0;
    virtual void mouseMoved(const QRect &panelRectangle, const QPoint &location) = 0;
    virtual void mouseButtonReleased(const QRect &panelRectangle, Qt::MouseButton button, const QPoint &location) = 0;
    virtual void mouseWheelMoved(const QRect &panelRectangle, int wheelDelta) = 0;

    // Camera transform
    virtual void applyTransformGL() = 0;
    virtual QVector3D cameraPosition() const = 0;
    virtual double zoom() const = 0;

    // Movement
    virtual void moveForward(double amount) = 0;
    virtual void strafeRight(double amount) = 0;
};

#endif // RENDER_VIEW_CAMERA_H

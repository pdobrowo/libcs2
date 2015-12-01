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

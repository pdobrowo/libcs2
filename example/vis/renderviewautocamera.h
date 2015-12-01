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
#ifndef RENDER_VIEW_AUTO_BALL_CAMERA_H
#define RENDER_VIEW_AUTO_BALL_CAMERA_H

#include "renderviewarcballcamera.h"
#include <QVector3D>
#include <QMatrix3x3>
#include <QObject>

class QTimer;

class RenderViewAutoCamera
    : public RenderViewArcBallCamera
{
    Q_OBJECT

private:
    // timer
    QTimer *m_timer;

private slots:
    void onTimeout();

public:
    RenderViewAutoCamera();

signals:
    void needsUpdate();
};

#endif // RENDER_VIEW_AUTO_BALL_CAMERA_H

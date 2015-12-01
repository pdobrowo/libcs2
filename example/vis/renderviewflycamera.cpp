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
#include "renderviewflycamera.h"
#include <QtOpenGL/QGLWidget>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>

namespace // anonymous
{
static const float EPSILON_MAX_PITCH = 0.01f;
} // namespace anonymous

RenderViewFlyCamera::RenderViewFlyCamera()
    : m_dragLeft(false),
      m_dragLeftBeginYaw(0.0f),
      m_dragLeftBeginPitch(0.0f),
      m_dragMiddle(false),
      m_yaw(M_PI),
      m_pitch(0.0f),
      m_sensitivity(0.002f),
      m_moveSensitivity(0.01f),
      m_position(QVector3D(5.0f, 0.0f, 1.5f))
{
}

RenderViewFlyCamera::~RenderViewFlyCamera()
{
}

void RenderViewFlyCamera::mouseButtonPressed(const QRect &panelRectangle, Qt::MouseButton buttons, const QPoint &location)
{
    (void)panelRectangle;

    if (buttons == Qt::LeftButton)
    {
        m_dragLeft = true;

        m_dragLeftBeginYaw = m_yaw;
        m_dragLeftBeginPitch = m_pitch;
        m_dragLeftBeginLocation = location;

        return;
    }

    if (buttons == Qt::MidButton)
    {
        m_dragMiddle = true;

        m_dragMiddleBeginPosition = m_position;
        m_dragMiddleBeginLocation = location;

        return;
    }
}

void RenderViewFlyCamera::mouseMoved(const QRect &panelRectangle, const QPoint &location)
{
    (void)panelRectangle;

    if (m_dragLeft)
    {
        QPoint deltaLocation = QPoint(location.x() - m_dragLeftBeginLocation.x(),
                                      location.y() - m_dragLeftBeginLocation.y());

        m_yaw = m_dragLeftBeginYaw - deltaLocation.x() * m_sensitivity;
        m_pitch = m_dragLeftBeginPitch + deltaLocation.y() * m_sensitivity;

        if (m_pitch > M_PI / 2 - EPSILON_MAX_PITCH)
            m_pitch = M_PI / 2 - EPSILON_MAX_PITCH;

        if (m_pitch < -M_PI / 2 + EPSILON_MAX_PITCH)
            m_pitch = -M_PI / 2 + EPSILON_MAX_PITCH;

        return;
    }

    if (m_dragMiddle)
    {
        QPoint deltaLocation = QPoint(location.x() - m_dragMiddleBeginLocation.x(),
                                      location.y() - m_dragMiddleBeginLocation.y());

        m_position = m_dragMiddleBeginPosition + forwardVector() * -deltaLocation.y() * m_moveSensitivity;

        return;
    }
}

void RenderViewFlyCamera::mouseButtonReleased(const QRect &panelRectangle, Qt::MouseButton buttons, const QPoint &location)
{
    (void)panelRectangle;

    if (buttons == Qt::LeftButton && m_dragLeft)
    {
        QPoint delta_position = QPoint(location.x() - m_dragLeftBeginLocation.x(),
                                       location.y() - m_dragLeftBeginLocation.y());

        m_yaw = m_dragLeftBeginYaw - delta_position.x() * m_sensitivity;
        m_pitch = m_dragLeftBeginPitch + delta_position.y() * m_sensitivity;

        m_dragLeft = false;

        if (m_pitch > M_PI / 2 - EPSILON_MAX_PITCH)
            m_pitch = M_PI / 2 - EPSILON_MAX_PITCH;

        if (m_pitch < -M_PI / 2 + EPSILON_MAX_PITCH)
            m_pitch = -M_PI / 2 + EPSILON_MAX_PITCH;

        return;
    }

    if (buttons == Qt::MidButton && m_dragMiddle)
    {
        QPoint deltaLocation = QPoint(location.x() - m_dragMiddleBeginLocation.x(),
                                      location.y() - m_dragMiddleBeginLocation.y());

        m_position = m_dragMiddleBeginPosition + forwardVector() * -deltaLocation.y() * m_moveSensitivity;

        m_dragMiddle = false;

        return;
    }
}

void RenderViewFlyCamera::mouseWheelMoved(const QRect &panelRectangle, int wheelDelta)
{
    (void)panelRectangle;

    // Update camera view
    m_position += forwardVector() * wheelDelta * m_moveSensitivity;
}

QVector3D RenderViewFlyCamera::forwardVector()
{
    return sphericalVector(m_yaw, m_pitch);
}

QVector3D RenderViewFlyCamera::strafeRightVector()
{
    return sphericalVector(m_yaw - M_PI_2, 0);
}

QVector3D RenderViewFlyCamera::sphericalVector(float yaw, float pitch)
{
    return QVector3D(cos(pitch) * cos(yaw),
                       cos(pitch) * sin(yaw),
                       -sin(pitch));
}

void RenderViewFlyCamera::applyTransformGL()
{
    // Reset modelview
    glLoadIdentity();

    // Apply camera view
    QVector3D lookAt = m_position + forwardVector();

    gluLookAt(m_position.x(), m_position.y(), m_position.z(),
              lookAt.x(), lookAt.y(), lookAt.z(),
              0.0f, 0.0f, 1.0f);
}


QVector3D RenderViewFlyCamera::cameraPosition() const
{
    return QVector3D(m_position.x(), m_position.y(), m_position.z());
}

double RenderViewFlyCamera::zoom() const
{
    return 1.0;
}

void RenderViewFlyCamera::moveForward(double amount)
{
    // Update camera view
    m_position += forwardVector() * m_moveSensitivity * amount;
}

void RenderViewFlyCamera::strafeRight(double amount)
{
    // Update camera view
    m_position += strafeRightVector() * m_moveSensitivity * amount;
}

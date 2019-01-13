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

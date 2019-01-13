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
#include "renderviewarcballcamera.h"
#include <QQuaternion>
#include <QGLWidget>
#include <qmath.h>
#include <cmath>

namespace // anonymous
{
static const double TRACKBALL_SIZE = 0.8;
static const double MOVE_SENSITIVITY = 0.02;

QMatrix3x3 matrixFromQuaternion(const QQuaternion &quaternion)
{
    QMatrix3x3 m;

    qreal ww = quaternion.scalar() * quaternion.scalar();
    qreal xx = quaternion.x() * quaternion.x();
    qreal yy = quaternion.y() * quaternion.y();
    qreal zz = quaternion.z() * quaternion.z();
    qreal inv = 1 / (xx + yy + zz + ww);
    qreal a, b;

    m(0, 0) = ( xx - yy - zz + ww) * inv;
    m(1, 1) = (-xx + yy - zz + ww) * inv;
    m(2, 2) = (-xx - yy + zz + ww) * inv;

    a = quaternion.x() * quaternion.y();
    b = quaternion.z() * quaternion.scalar();
    m(1, 0) = 2.0 * (a + b) * inv;
    m(0, 1) = 2.0 * (a - b) * inv;

    a = quaternion.x() * quaternion.z();
    b = quaternion.y() * quaternion.scalar();
    m(2, 0) = 2.0 * (a - b) * inv;
    m(0, 2) = 2.0 * (a + b) * inv;

    a = quaternion.y() * quaternion.z();
    b = quaternion.x() * quaternion.scalar();
    m(2, 1) = 2.0 * (a + b) * inv;
    m(1, 2) = 2.0 * (a - b) * inv;

    return m;
}

QMatrix3x3 matrixRotationX(qreal angle)
{
    QMatrix3x3 m;

    qreal s = qSin(angle);
    qreal c = qCos(angle);

    m(1, 1) = c;
    m(1, 2) = -s;
    m(2, 1) = s;
    m(2, 2) = c;

    return m;
}

qreal RadToDegree(qreal a)
{
    return a * 180.0 / 3.14159265358979323846;
}

} // namespace anonymous

RenderViewArcBallCamera::RenderViewArcBallCamera()
    : m_dragLeft(false),
      m_dragMiddle(false),
      m_rotation(QMatrix3x3()),
      m_zoom(1.0f)
{
    m_rotation = matrixRotationX(3.14159265358979323846 / 3.0);
}

void RenderViewArcBallCamera::mouseButtonPressed(const QRect& panelRectangle, Qt::MouseButton buttons, const QPoint& location)
{
    if (buttons == Qt::LeftButton)
    {
        m_dragLeft = true;

        m_dragLeftBeginNormalizedLocation = normalizeCoordinates(location, panelRectangle);
        m_dragLeftBeginRotation = m_rotation;

        return;
    }

    if (buttons == Qt::MidButton)
    {
        m_dragMiddle = true;

        m_dragMiddleBeginLocation = location;
        m_dragMiddleBeginScale = m_zoom;

        return;
    }
}

void RenderViewArcBallCamera::mouseMoved(const QRect& panelRectangle, const QPoint& location)
{
    if (m_dragLeft)
    {
        QPointF normalizedLocation = normalizeCoordinates(location, panelRectangle);
        QQuaternion quaternion = simulateArcBall(m_dragLeftBeginNormalizedLocation, normalizedLocation);

        m_rotation = m_dragLeftBeginRotation * matrixFromQuaternion(quaternion);

        return;
    }

    if (m_dragMiddle)
    {
        QPoint deltaLocation = QPoint(location.x() - m_dragMiddleBeginLocation.x(),
                                      location.y() - m_dragMiddleBeginLocation.y());

        float newScale = m_dragMiddleBeginScale + MOVE_SENSITIVITY * deltaLocation.y();

        if (newScale > 0)
            m_zoom = newScale;

        return;
    }
}

void RenderViewArcBallCamera::mouseButtonReleased(const QRect& panelRectangle, Qt::MouseButton buttons, const QPoint& location)
{
    if (buttons == Qt::LeftButton && m_dragLeft)
    {
        QPointF normalizedLocation = normalizeCoordinates(location, panelRectangle);
        QQuaternion quaternion = simulateArcBall(m_dragLeftBeginNormalizedLocation, normalizedLocation);

        m_rotation = m_dragLeftBeginRotation * matrixFromQuaternion(quaternion);

        m_dragLeft = false;

        return;
    }

    if (buttons == Qt::MidButton && m_dragMiddle)
    {
        QPoint deltaLocation = QPoint(location.x() - m_dragMiddleBeginLocation.x(),
                                      location.y() - m_dragMiddleBeginLocation.y());

        float newScale = m_dragMiddleBeginScale + MOVE_SENSITIVITY * deltaLocation.y();

        if (newScale > 0)
            m_zoom = newScale;

        m_dragMiddle = false;

        return;
    }
}

void RenderViewArcBallCamera::mouseWheelMoved(const QRect& panelRectangle, int wheelDelta)
{
    (void)panelRectangle;

    float zoom = m_zoom + MOVE_SENSITIVITY * -wheelDelta;

    if (zoom > 0)
        m_zoom = zoom;
}

QPointF RenderViewArcBallCamera::normalizeCoordinates(const QPoint &point, const QRect &rect)
{
    return QPointF(2.0 * (point.x() - rect.left()) / (rect.width() - 1) - 1.0,
                   2.0 * (rect.bottom() - point.y() - 1) / (rect.height() - 1) - 1.0);
}

QQuaternion RenderViewArcBallCamera::simulateArcBall(const QPointF &begin, const QPointF &end)
{
    if (begin == end)
        return QQuaternion();

    QVector3D sphereBegin(begin.x(), begin.y(), mapToSphere(TRACKBALL_SIZE, begin.x(), begin.y()));
    QVector3D sphereEnd(end.x(), end.y(), mapToSphere(TRACKBALL_SIZE, end.x(), end.y()));
    QVector3D axis = QVector3D::crossProduct(sphereEnd, sphereBegin);
    QVector3D deltaVector = sphereBegin - sphereEnd;

    double delta = deltaVector.length() / (2.0 * TRACKBALL_SIZE);

    if (delta > 1.0)
        delta = 1.0;

    if (delta < -1.0)
        delta = -1.0;

    return QQuaternion::fromAxisAndAngle(axis, RadToDegree(2.0 * asin(delta)));
}

double RenderViewArcBallCamera::mapToSphere(qreal r, qreal x, qreal y)
{
    qreal d, t, z;

    d = qSqrt(x * x + y * y);

    if (d < r * 0.70710678118654752330)
    {
        // inside sphere
        z = qSqrt(r * r - d * d);
    }
    else
    {
        // on hyperbola
        t = r / 1.41421356237309504880;
        z = t * t / d;
    }

    return z;
}

void RenderViewArcBallCamera::applyTransformGL()
{
    // Reset modelview matrix
    glLoadIdentity();

    // Create default look at matrix
    glTranslated(0.0, 0.0, -5.0 * m_zoom);

    // Convert matrix to OpenGL
    static double matrix[16]; // static prevents crash on some drivers

    for (int i = 0; i < 16; ++i)
        matrix[i] = 0.0;

    matrix[15] = 1.0;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            matrix[i * 4 + j] = m_rotation(i, j);

    // Apply arcball rotation
    glMultMatrixd(matrix);
}

QVector3D RenderViewArcBallCamera::cameraPosition() const
{
    return QVector3D(0.0, 0.0, 5.0 * m_zoom);
}

double RenderViewArcBallCamera::zoom() const
{
    return m_zoom;
}

void RenderViewArcBallCamera::moveForward(double amount)
{
    // does nothing
    (void)amount;
}

void RenderViewArcBallCamera::strafeRight(double amount)
{
    // does nothing
    (void)amount;
}

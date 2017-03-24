/**
 * Copyright (c) 2015-2017 Przemysław Dobrowolski
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
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include "renderview.h"
#include "renderviewflycamera.h"
#include "renderviewarcballcamera.h"
#include <QPaintEvent>
#include <QMouseEvent>
#include <QVector>
#include <QApplication>
#include <QTimer>
#include <stdlib.h>
#include <cmath>

namespace // anonymous
{
static QColor OBJECT_COLOR[] =
{
    QColor(Qt::red),
    QColor(Qt::green),
    QColor(Qt::blue),
    QColor(Qt::cyan),
    QColor(Qt::magenta),
    QColor(Qt::gray)
};

static const size_t NUM_OBJECT_COLORS = sizeof(OBJECT_COLOR) / sizeof(OBJECT_COLOR[0]);

void applyRotationGL(const QQuaternion &quaternion)
{
    double x2 = quaternion.x() * quaternion.x();
    double y2 = quaternion.y() * quaternion.y();
    double z2 = quaternion.z() * quaternion.z();
    double xy = quaternion.x() * quaternion.y();
    double xz = quaternion.x() * quaternion.z();
    double yz = quaternion.y() * quaternion.z();
    double wx = quaternion.scalar() * quaternion.x();
    double wy = quaternion.scalar() * quaternion.y();
    double wz = quaternion.scalar() * quaternion.z();

    double matrix[16] =
    {
        1.0 - 2.0 * (y2 + z2),  2.0 * (xy - wz),        2.0 * (xz + wy),        0.0,
        2.0 * (xy + wz),        1.0 - 2.0 * (x2 + z2),  2.0 * (yz - wx),        0.0,
        2.0 * (xz - wy),        2.0 * (yz + wx),        1.0 - 2.0 * (x2 + y2),  0.0,
        0.0,                    0.0,                    0.0,                    1.0
    };

    glMultMatrixd(matrix);
}
} // namespace anonymous

RenderView::RenderView(QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags f)
    : QGLWidget(parent, shareWidget, f)
{
    ctor();
}

RenderView::RenderView(QGLContext *context, QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags f)
    : QGLWidget(context, parent, shareWidget, f)
{
    ctor();
}

RenderView::RenderView(const QGLFormat &format, QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags f)
    : QGLWidget(format, parent, shareWidget, f)
{
    ctor();
}

void RenderView::ctor()
{
    m_nextSuggestedColor = 0;
    m_viewModeOptions = 0;

    setMouseTracking(true);

    // Default values
    m_caption = QString("unknown");
    m_textColor = QColor(255, 255, 128);
    m_textFont = QFont(QApplication::font().family(), 10);

    // Default camera
    m_camera.reset(new RenderViewArcBallCamera());

    // update view timer
    QTimer *t = new QTimer(this);
    connect(t, SIGNAL(timeout()), this, SLOT(updateGL()));
    t->setSingleShot(false);
    t->start(1000 / 60); // display at most 60 fps
}

RenderView::~RenderView()
{
}

void RenderView::mousePressEvent(QMouseEvent *event)
{
    QWidget::mousePressEvent(event);
    m_camera->mouseButtonPressed(rect(), event->button(), event->pos());
}

void RenderView::mouseMoveEvent(QMouseEvent *event)
{
    QWidget::mouseMoveEvent(event);
    m_camera->mouseMoved(rect(), event->pos());
}

void RenderView::mouseReleaseEvent(QMouseEvent *event)
{
    QWidget::mouseReleaseEvent(event);
    m_camera->mouseButtonReleased(rect(), event->button(), event->pos());
}

void RenderView::wheelEvent(QWheelEvent *event)
{
    QGLWidget::wheelEvent(event);

    // Handle mouse wheel
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;

    m_camera->mouseWheelMoved(rect(), numSteps);
}

QString RenderView::caption() const
{
    return m_caption;
}

void RenderView::setCaption(const QString& value)
{
    m_caption = value;
}

QFont RenderView::textFont() const
{
    return m_textFont;
}

QColor RenderView::nextSuggestedColor()
{
    return OBJECT_COLOR[(m_nextSuggestedColor++) % NUM_OBJECT_COLORS];
}

void RenderView::setTextFont(const QFont& value)
{
    m_textFont = value;

    // update meshes
    m_grid->setTextFont(m_textFont);
    m_grid->reset();
}

void RenderView::initializeGL()
{
    // init extensions
    glewInit();

    // init perpixel shader
    m_perPixelLightingShader.reset(new Shader("perpixel.vert", "perpixel.frag"));

    // Set perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(75.0f, static_cast<GLfloat>(width()) / static_cast<GLfloat>(height()), 0.1f, 1024.0f);

    // Set viewport
    glViewport(0, 0, static_cast<GLsizei>(width()), static_cast<GLsizei>(height()));

    // Set modelview
    glMatrixMode(GL_MODELVIEW);

    // Enable culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Front face definition
    glFrontFace(GL_CCW);

    // Disable texturing
    glDisable(GL_TEXTURE_2D);

    // Smooth shading
    glShadeModel(GL_SMOOTH);

    // Clear mode
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClearDepth(1.0f);

    // Depth buffer
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Somewhere in the initialization part of your program?
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Create light components
    const GLfloat lightAmbient[] = { 0.2, 0.2, 0.2, 1.0 };
    const GLfloat lightDiffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    const GLfloat lightSpecular[] = { 1.0, 1.0, 1.0, 1.0 };

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    // Material
    GLfloat materialShininess[] = { 50 };

    GLfloat material_diffuse[] = { 1.0, 0.0, 0.0, 1.0 };
    GLfloat materialAmbient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat materialSpecular[] = { 1.0, 1.0, 1.0, 1.0 };

    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, materialAmbient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, materialShininess);

    // Nice perspective
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Multisampling
    if (0)
    {
        glEnable(GL_MULTISAMPLE);
        GLint bufs;
        GLint samples;
        glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
        glGetIntegerv(GL_SAMPLES, &samples);
        qDebug("Multisampling: %i buffers and %i samples", bufs, samples);
    }

    // prerender meshes
    m_grid.reset(new GridMesh(this));
    m_grid->setTextColor(m_textColor);
    m_grid->setTextFont(m_textFont);
}

void RenderView::resizeGL(int, int)
{
    // Set perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<GLfloat>(width()) / static_cast<GLfloat>(height()), 0.01f, 1024.0f);

    // Set viewport
    glViewport(0, 0, static_cast<GLsizei>(width()), static_cast<GLsizei>(height()));

    // Set modelview
    glMatrixMode(GL_MODELVIEW);
}

void RenderView::paintGL()
{
    // clear view
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // draw gradient background
    if (!isViewModeOption(ViewMode::ModelOnly))
        drawGradientBackground();

    // setup camera
    m_camera->applyTransformGL();

    // setup lighting
    drawLights();

    // draw axies
    if (!isViewModeOption(ViewMode::ModelOnly))
        drawAxies();

    // culling enabled for conf-spaces, but not for scenes
    if (isViewModeOption(ViewMode::Culling))
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);

    // draw contents
    renderMeshes();

    // Draw panel name
    if (!isViewModeOption(ViewMode::ModelOnly))
    {
        qglColor(m_textColor);
        renderText(5, 15, m_caption, m_textFont);

        QString percent;
        percent.sprintf("%.0f%%", m_camera->zoom() * 100);
        renderText(width() - 45, 15, percent, m_textFont);
    }
}

void RenderView::drawGradientBackground()
{
    glDisable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Autodesk Maya 2011 background
    glBegin(GL_QUADS);
    glColor3ub(125, 145, 165);
    glVertex2i( 1, 1);
    glVertex2i(-1, 1);

    glColor3ub(26, 27, 29);
    glVertex2i(-1, -1);
    glVertex2i( 1, -1);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}

void RenderView::mouseDoubleClickEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    emit toggleFullScreenTriggered();
}

void RenderView::drawAxies()
{
    glDisable(GL_LIGHTING);
    m_grid->render();
}

void RenderView::drawLights()
{
    GLfloat lightPositionA[] = {  20.0,  20.0,  20.0, 1.0 };
    GLfloat lightPositionB[] = { -20.0, -20.0, -20.0, 1.0 };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPositionA);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPositionB);
}

void RenderView::addTriangleList(TriangleListPtr triangleList, QColor color)
{
    if (m_triangleLists.find(triangleList) != m_triangleLists.end())
        return;

    TriangleListDataPtr data(new TriangleListData(this, triangleList, color));
    m_triangleLists[triangleList] = data;
}

void RenderView::renderMeshes()
{
    // draw bodies
    if (isViewModeOption(ViewMode::Wireframe))
    {
        // wireframe rendering
        glDisable(GL_LIGHTING);
    }
    else
    {
        // normal rendering
        glEnable(GL_LIGHTING);
        m_perPixelLightingShader->bind();
    }

    // draw triangle lists
    for (TriangleLists::const_iterator it = m_triangleLists.begin(); it != m_triangleLists.end(); ++it)
        it->second->render(isViewModeOption(ViewMode::Wireframe));

    if (isViewModeOption(ViewMode::Wireframe))
    {
        glEnable(GL_LIGHTING);
        m_perPixelLightingShader->bind();
    }

    m_perPixelLightingShader->unbind();
    glDisable(GL_LIGHTING);

    // draw outlines
    if (isViewModeOption(ViewMode::Outlines))
        for (TriangleLists::const_iterator it = m_triangleLists.begin(); it != m_triangleLists.end(); ++it)
            it->second->renderOutlines();

    // draw normals
    if (isViewModeOption(ViewMode::Normals))
        for (TriangleLists::const_iterator it = m_triangleLists.begin(); it != m_triangleLists.end(); ++it)
            it->second->renderNormals();
}

void RenderView::setTriangleListVisible(TriangleListPtr triangleList, bool visible)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return;

    m_triangleLists[triangleList]->setVisible(visible);
}

void RenderView::keyPressEvent(QKeyEvent *event)
{
    const double MOVE_AMOUNT = 4.0;

    switch (event->key())
    {
    case Qt::Key_Up:
    case Qt::Key_W:
        m_camera->moveForward(MOVE_AMOUNT);
        return;

    case Qt::Key_Down:
    case Qt::Key_S:
        m_camera->moveForward(-MOVE_AMOUNT);
        return;

    case Qt::Key_Right:
    case Qt::Key_D:
        m_camera->strafeRight(MOVE_AMOUNT);
        return;

    case Qt::Key_Left:
    case Qt::Key_A:
        m_camera->strafeRight(-MOVE_AMOUNT);
        return;
    }

    // call base member
    QGLWidget::keyPressEvent(event);
}

void RenderView::setRenderViewCamera(RenderViewCamera *camera)
{
    m_camera.reset(camera);
}

ViewMode::Options RenderView::viewModeOptions() const
{
    return m_viewModeOptions;
}

void RenderView::setViewModeOptions(ViewMode::Options options)
{
    m_viewModeOptions = options;
}

bool RenderView::isViewModeOption(ViewMode::Option option) const
{
    return !!(m_viewModeOptions & option);
}

void RenderView::enableViewModeOption(ViewMode::Option option)
{
    m_viewModeOptions |= option;
}

void RenderView::disableViewModeOption(ViewMode::Option option)
{
    m_viewModeOptions &= ~option;
}

void RenderView::setViewModeOption(ViewMode::Option option, bool state)
{
    if (state)
        enableViewModeOption(option);
    else
        disableViewModeOption(option);
}

void RenderView::toogleViewModeOption(ViewMode::Option option)
{
    if (isViewModeOption(option))
        disableViewModeOption(option);
    else
        enableViewModeOption(option);
}

void RenderView::removeTriangleList(TriangleListPtr triangleList)
{
    TriangleLists::iterator it = m_triangleLists.find(triangleList);

    if (it == m_triangleLists.end())
        return;

    m_triangleLists.erase(it);
}

bool RenderView::isTriangleListVisible(TriangleListPtr triangleList)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return false;

    return m_triangleLists[triangleList]->isVisible();
}

void RenderView::setTriangleListTranslation(TriangleListPtr triangleList, const QVector3D &translation)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return;

    m_triangleLists[triangleList]->setTranslation(translation);
}

QVector3D RenderView::TriangleListTranslation(TriangleListPtr triangleList)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return QVector3D();

    return m_triangleLists[triangleList]->translation();
}

void RenderView::setTriangleListQuaternion(TriangleListPtr triangleList, const QQuaternion &quaternion)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return;

    m_triangleLists[triangleList]->setQuaternion(quaternion);
}

QQuaternion RenderView::TriangleListQuaternion(TriangleListPtr triangleList)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return QQuaternion();

    return m_triangleLists[triangleList]->quaternion();
}

void RenderView::setTriangleListColor(TriangleListPtr triangleList, const QColor &color)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return;

    m_triangleLists[triangleList]->setColor(color);
}

QColor RenderView::TriangleListColor(TriangleListPtr triangleList)
{
    if (m_triangleLists.find(triangleList) == m_triangleLists.end())
        return QColor();

    return m_triangleLists[triangleList]->color();
}

void RenderView::removeAllObjects()
{
    m_triangleLists.clear();
}

TriangleListData::TriangleListData(QGLWidget *gl, TriangleListPtr triangleList, QColor color)
    : m_renderObject(gl, triangleList),
      m_color(color)
{
}

void TriangleListData::render(bool wireframe)
{
    // check visibility
    if (!m_visible)
        return;

    // setup material
    //glColor3ub(m_color.red(), m_color.green(), m_color.blue());
    GLfloat material_diffuse[] = { static_cast<GLfloat>(m_color.redF()), static_cast<GLfloat>(m_color.greenF()), static_cast<GLfloat>(m_color.blueF()), 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);

    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // draw geometry
    glPushMatrix();
    glTranslated(m_translation.x(), m_translation.y(), m_translation.z());
    applyRotationGL(m_quaternion);
    m_renderObject.render();
    glPopMatrix();

    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void TriangleListData::renderOutlines()
{
    // check visibility
    if (!m_visible)
        return;

    // draw outlines
    glPushMatrix();
    glTranslated(m_translation.x(), m_translation.y(), m_translation.z());
    applyRotationGL(m_quaternion);
    m_renderObject.renderOutlines();
    glPopMatrix();
}

void TriangleListData::renderNormals()
{
    // check visibility
    if (!m_visible)
        return;

    // draw normals
    glPushMatrix();
    glTranslated(m_translation.x(), m_translation.y(), m_translation.z());
    applyRotationGL(m_quaternion);
    m_renderObject.renderNormals();
    glPopMatrix();
}

void TriangleListData::setColor(QColor color)
{
    m_color = color;
}

QColor TriangleListData::color() const
{
    return m_color;
}

void MeshData::setVisible(bool visible)
{
    m_visible = visible;
}

bool MeshData::isVisible() const
{
    return m_visible;
}

void MeshData::setTranslation(const QVector3D &translation)
{
    m_translation = translation;
}

QVector3D MeshData::translation() const
{
    return m_translation;
}

void MeshData::setQuaternion(const QQuaternion &quaternion)
{
    m_quaternion = quaternion;
}

QQuaternion MeshData::quaternion() const
{
    return m_quaternion;
}

MeshData::MeshData()
    : m_visible(true),
      m_translation(QVector3D()),
      m_quaternion(QQuaternion())
{
}

MeshData::~MeshData()
{
}

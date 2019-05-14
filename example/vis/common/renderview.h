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
#ifndef RENDERVIEW_H
#define RENDERVIEW_H

#include "renderviewcamera.h"
#include "shader.h"
#include "gridmesh.h"
#include "linelistmesh.h"
#include "trianglelistmesh.h"
#include <QVector3D>
#include <QQuaternion>
#include <QWidget>
#include <QColor>
#include <QPoint>
#include <QString>
#include <QPainter>
#include <QLabel>
#include <QPicture>
#include <QImage>
#include <QGLWidget>
#include <boost/scoped_ptr.hpp>
#include <boost/cstdint.hpp>
#include <map>
#include <set>

class QPaintEvent;
class QMouseEvent;

class MeshData
{
public:
    MeshData();
    virtual ~MeshData();

    void        setVisible(bool visible);
    bool        isVisible() const;

    void        setTranslation(const QVector3D &translation);
    QVector3D   translation() const;

    void        setQuaternion(const QQuaternion &quaternion);
    QQuaternion quaternion() const;

protected:
    bool        m_visible;
    QVector3D   m_translation;
    QQuaternion m_quaternion;
};

class LineListData
    : public MeshData
{
public:
    LineListData(QGLWidget *gl,
                 LineListPtr lineList,
                 QColor color);

    void    render();

    void    setColor(QColor color);
    QColor  color() const;

private:
    LineListMesh    m_renderObject;
    QColor          m_color;
};

typedef boost::shared_ptr<LineListData> LineListDataPtr;
typedef std::map<LineListPtr, LineListDataPtr> LineLists;

class TriangleListData
    : public MeshData
{
public:
    TriangleListData(QGLWidget *gl,
                     TriangleListPtr triangleList,
                     QColor color);

    void    render(bool wireframe);
    void    renderOutlines();
    void    renderNormals();

    void    setColor(QColor color);
    QColor  color() const;

private:
    TriangleListMesh    m_renderObject;
    QColor              m_color;
};

typedef boost::shared_ptr<TriangleListData> TriangleListDataPtr;
typedef std::map<TriangleListPtr, TriangleListDataPtr> TriangleLists;

class ViewMode
{
public:
    enum Option
    {
        Wireframe       = 1,
        Culling         = 2,
        ModelOnly       = 4,
        Outlines        = 8,
        Normals         = 16
    };
    Q_DECLARE_FLAGS(Options, Option)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ViewMode::Options)

class RenderView
    : public QGLWidget
{
    Q_OBJECT

private:
    // view title
    QString m_caption;

    // view colors
    QColor m_textColor;

    // view font
    QFont m_textFont;

    // camera
    boost::scoped_ptr<RenderViewCamera> m_camera;

    // meshes
    boost::scoped_ptr<GridMesh> m_grid;

    // axies
    void drawAxies();

    // lights
    void drawLights();

    // gradient background
    void drawGradientBackground();

    // render objects
    TriangleLists m_triangleLists;
    LineLists m_lineLists;

    size_t m_nextSuggestedColor;

    // meshes
    void renderMeshes();

    // view modes
    ViewMode::Options m_viewModeOptions;

    // shaders
    boost::scoped_ptr<Shader> m_shaderLighting;

    void ctor();

protected:
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);

    virtual void mouseDoubleClickEvent(QMouseEvent *event);

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

public:
    explicit RenderView(QWidget* parent = nullptr, const QGLWidget *shareWidget = nullptr, Qt::WindowFlags f = nullptr);
    explicit RenderView(QGLContext *context, QWidget *parent = nullptr, const QGLWidget *shareWidget = nullptr, Qt::WindowFlags f = nullptr);
    explicit RenderView(const QGLFormat &format, QWidget *parent = nullptr, const QGLWidget *shareWidget = nullptr, Qt::WindowFlags f = nullptr);
    virtual ~RenderView();

    // properties
    void setCaption(const QString& value);
    QString caption() const;

    void setTextFont(const QFont& value);
    QFont textFont() const;

    // colors
    QColor nextSuggestedColor();

    // triangle lists
    void addTriangleList(TriangleListPtr triangleList, QColor color);
    void removeTriangleList(TriangleListPtr triangleList);

    void setTriangleListVisible(TriangleListPtr triangleList, bool visible);
    bool isTriangleListVisible(TriangleListPtr triangleList);

    void setTriangleListTranslation(TriangleListPtr triangleList, const QVector3D &translation);
    QVector3D TriangleListTranslation(TriangleListPtr triangleList);

    void setTriangleListQuaternion(TriangleListPtr triangleList, const QQuaternion &rotation);
    QQuaternion TriangleListQuaternion(TriangleListPtr triangleList);

    void setTriangleListColor(TriangleListPtr triangleList, const QColor &color);
    QColor TriangleListColor(TriangleListPtr triangleList);

    // line lists
    void addLineList(LineListPtr lineList, QColor color);
    void removeLineList(LineListPtr lineList);

    void setLineListVisible(LineListPtr lineList, bool visible);
    bool isLineListVisible(LineListPtr lineList);

    void setLineListTranslation(LineListPtr lineList, const QVector3D &translation);
    QVector3D LineListTranslation(LineListPtr lineList);

    void setLineListQuaternion(LineListPtr lineList, const QQuaternion &rotation);
    QQuaternion LineListQuaternion(LineListPtr lineList);

    void setLineListColor(LineListPtr lineList, const QColor &color);
    QColor LineListColor(LineListPtr lineList);

    // cleanup
    void removeAllObjects();

    // camera
    void setRenderViewCamera(RenderViewCamera *camera);

    // view modes
    ViewMode::Options viewModeOptions() const;
    void setViewModeOptions(ViewMode::Options options);

    bool isViewModeOption(ViewMode::Option option) const;
    void enableViewModeOption(ViewMode::Option option);
    void disableViewModeOption(ViewMode::Option option);
    void setViewModeOption(ViewMode::Option option, bool state);
    void toogleViewModeOption(ViewMode::Option option);

signals:
    void toggleFullScreenTriggered();
};

#endif // RENDERVIEW_H

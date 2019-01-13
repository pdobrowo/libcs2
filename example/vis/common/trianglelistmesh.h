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
#ifndef TRIANGLELISTMESH_H
#define TRIANGLELISTMESH_H

#include "mesh.h"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <QVector3D>

class Triangle
{
public:
    Triangle()
    {
    }

    Triangle(const QVector3D &vertexA, const QVector3D &vertexB, const QVector3D &vertexC,
             const QVector3D &normalA, const QVector3D &normalB, const QVector3D &normalC,
             const QVector3D &sharedNormalA, const QVector3D &sharedNormalB, const QVector3D &sharedNormalC)
    {
        m_vertex[0] = vertexA;
        m_vertex[1] = vertexB;
        m_vertex[2] = vertexC;

        m_normal[0] = normalA;
        m_normal[1] = normalB;
        m_normal[2] = normalC;

        m_sharedNormal[0] = sharedNormalA;
        m_sharedNormal[1] = sharedNormalB;
        m_sharedNormal[2] = sharedNormalC;
    }

    const QVector3D &vertex(size_t i) const
    {
        return m_vertex[i];
    }

    const QVector3D &normal(size_t i) const
    {
        return m_normal[i];
    }

    const QVector3D &sharedNormal(size_t i) const
    {
        return m_sharedNormal[i];
    }

    Triangle flipped() const
    {
        return Triangle(m_vertex[2], m_vertex[1], m_vertex[0],
                        -m_normal[2], -m_normal[1], -m_normal[0],
                        -m_sharedNormal[2], -m_sharedNormal[1], -m_sharedNormal[0]);
    }

private:
    QVector3D m_vertex[3], m_normal[3], m_sharedNormal[3];
};

typedef std::vector<Triangle> TriangleList;
typedef boost::shared_ptr<TriangleList> TriangleListPtr;

class TriangleListMesh
    : public Mesh
{
public:
    TriangleListMesh(QGLWidget *gl, TriangleListPtr triangleList);

    void renderOutlines();
    void renderNormals();

private:
    TriangleListPtr m_triangleList;

    void drawTriangleList();

protected:
    virtual void drawMesh();
};

typedef boost::shared_ptr<TriangleListMesh> TriangleListMeshPtr;

#endif // TRIANGLELISTMESH_H

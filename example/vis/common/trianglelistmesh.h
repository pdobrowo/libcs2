/**
 * Copyright (c) 2015-2016 Przemysław Dobrowolski
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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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
        m_vertexA = m_vertexB = m_vertexC = QVector3D(0, 0, 0);
        m_normal = QVector3D::crossProduct(m_vertexB - m_vertexA, m_vertexC - m_vertexA);
    }

    Triangle(const QVector3D &vertexA, const QVector3D &vertexB, const QVector3D &vertexC)
    {
        m_vertexA = vertexA;
        m_vertexB = vertexB;
        m_vertexC = vertexC;
        m_normal = QVector3D::crossProduct(m_vertexB - m_vertexA, m_vertexC - m_vertexA);
    }

    const QVector3D &   vertex(int i) const
    {
        switch (i)
        {
        case 0: return m_vertexA;
        case 1: return m_vertexB;
        case 2: return m_vertexC;
        default: assert(0); return m_vertexA;
        }
    }

    const QVector3D &   normal() const
    {
        return m_normal;
    }

private:
    QVector3D m_vertexA, m_vertexB, m_vertexC;
    QVector3D m_normal;
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

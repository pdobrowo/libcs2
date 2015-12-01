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

private:
    TriangleListPtr m_triangleList;

    void drawTriangleList();

protected:
    virtual void drawMesh();
};

typedef boost::shared_ptr<TriangleListMesh> TriangleListMeshPtr;

#endif // TRIANGLELISTMESH_H

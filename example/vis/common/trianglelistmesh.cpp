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
#include "trianglelistmesh.h"
#include <QVector3D>
#include <GL/gl.h>

const double SCALE = 0.05;

TriangleListMesh::TriangleListMesh(QGLWidget *gl, TriangleListPtr triangleList)
    : Mesh(gl),
      m_triangleList(triangleList)
{
}

void TriangleListMesh::renderOutlines()
{
    glColor3ub(100, 100, 100);

    glBegin(GL_LINES);

    for (TriangleList::const_iterator iterator = m_triangleList->begin(); iterator != m_triangleList->end(); ++iterator)
    {
        const Triangle &triangle = *iterator;

        glVertex3d(triangle.vertex(0).x(), triangle.vertex(0).y(), triangle.vertex(0).z());
        glVertex3d(triangle.vertex(1).x(), triangle.vertex(1).y(), triangle.vertex(1).z());
        glVertex3d(triangle.vertex(1).x(), triangle.vertex(1).y(), triangle.vertex(1).z());
        glVertex3d(triangle.vertex(2).x(), triangle.vertex(2).y(), triangle.vertex(2).z());
        glVertex3d(triangle.vertex(2).x(), triangle.vertex(2).y(), triangle.vertex(2).z());
        glVertex3d(triangle.vertex(0).x(), triangle.vertex(0).y(), triangle.vertex(0).z());
    }

    glEnd();
}

void TriangleListMesh::renderNormals()
{
    glColor3ub(255, 127, 0);

    glBegin(GL_LINES);

    for (TriangleList::const_iterator iterator = m_triangleList->begin(); iterator != m_triangleList->end(); ++iterator)
    {
        const Triangle &triangle = *iterator;

        for (size_t i = 0; i < 3; ++i)
        {
            glVertex3d(triangle.vertex(i).x(),
                       triangle.vertex(i).y(),
                       triangle.vertex(i).z());

            glVertex3d(triangle.vertex(i).x() + SCALE * triangle.normal(i).x(),
                       triangle.vertex(i).y() + SCALE * triangle.normal(i).y(),
                       triangle.vertex(i).z() + SCALE * triangle.normal(i).z());
        }
    }

    glEnd();
}

void TriangleListMesh::drawMesh()
{
    drawTriangleList();
}

void TriangleListMesh::drawTriangleList()
{
    // draw faces
    glBegin(GL_TRIANGLES);

    for (TriangleList::const_iterator iterator = m_triangleList->begin(); iterator != m_triangleList->end(); ++iterator)
    {
        const Triangle &triangle = *iterator;

        for (size_t i = 0; i < 3; ++i)
        {
            glNormal3d(triangle.sharedNormal(i).x(), triangle.sharedNormal(i).y(), triangle.sharedNormal(i).z());
            glVertex3d(triangle.vertex(i).x(), triangle.vertex(i).y(), triangle.vertex(i).z());
        }
    }

    glEnd();
}

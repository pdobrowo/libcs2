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
#include "gridmesh.h"
#include <QGLWidget>

GridMesh::GridMesh(QGLWidget *gl)
    : Mesh(gl)
{
}

void GridMesh::render()
{
    Mesh::render();

    // Render axies descriptions
    m_gl->qglColor(m_textColor);
    m_gl->renderText(1.0, 0.0, 0.1, QString("X"), m_textFont);
    m_gl->renderText(0.0, 1.0, 0.1, QString("Y"), m_textFont);
    m_gl->renderText(0.0, 0.0, 1.1, QString("Z"), m_textFont);
}

void GridMesh::setTextColor(QColor color)
{
    m_textColor = color;
}

QColor GridMesh::textColor() const
{
    return m_textColor;
}

void GridMesh::setTextFont(QFont font)
{
    m_textFont = font;
}

QFont GridMesh::textFont() const
{
    return m_textFont;
}

void GridMesh::drawMesh()
{
    // Begin render lines
    glBegin(GL_LINES);

    // Draw zero grid
    static const int NUM_ZERO_GRID_LINES = 10;
    static const int ZERO_GRID_SIZE = 2;

    for (int i = -NUM_ZERO_GRID_LINES; i <= NUM_ZERO_GRID_LINES; ++i)
    {
        if (i == 0)
            glColor3ub(0, 0, 0);
        else
            glColor3ub(120, 123, 128);

        double pos = double(ZERO_GRID_SIZE * i) / double(NUM_ZERO_GRID_LINES);

        // X line
        glVertex3d(pos, -ZERO_GRID_SIZE, 0.0);
        glVertex3d(pos, ZERO_GRID_SIZE, 0.0);

        // Y line
        glVertex3d(-ZERO_GRID_SIZE, pos, 0.0);
        glVertex3d(ZERO_GRID_SIZE, pos, 0.0);
    }

    // Draw X axis
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.01);
    glVertex3d(1.0, 0.0, 0.01);

    // Draw Y axis
    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(0.0, 0.0, 0.01);
    glVertex3d(0.0, 1.0, 0.01);

    // Draw Z axis
    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 1.0);

    // Done render
    glEnd();
}

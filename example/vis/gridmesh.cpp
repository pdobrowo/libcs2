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

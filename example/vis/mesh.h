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
#ifndef MESH_H
#define MESH_H

#include <boost/noncopyable.hpp>
#include <QColor>

class QGLWidget;

class Mesh
    : private boost::noncopyable
{
public:
    Mesh(QGLWidget *gl);
    virtual ~Mesh();

    virtual void render();
    void reset();

protected:
    QGLWidget *     m_gl;

    virtual void drawMesh() = 0;

private:
    unsigned int    m_list;
};

void enableViewClipPlane();
void disableViewClipPlane();
void setColorGL(const QColor &color);

#endif // MESH_H

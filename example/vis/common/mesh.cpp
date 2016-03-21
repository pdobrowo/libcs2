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
#include "mesh.h"
#include <GL/gl.h>

Mesh::Mesh(QGLWidget *gl)
    : m_gl(gl),
      m_list(0)
{
    // use lazy initialization
}

Mesh::~Mesh()
{
    reset();
}

void Mesh::render()
{
    if (!m_list)
    {
        // create one display list
        m_list = glGenLists(1);

        // compile the display list
        glNewList(m_list, GL_COMPILE);
        drawMesh();
        glEndList();
    }

    // draw the display list
    glCallList(m_list);
}

void Mesh::reset()
{
    // delete list if initialized
    if (m_list)
    {
        glDeleteLists(m_list, 1);
        m_list = 0;
    }
}

GLdouble glDoubleSign(GLdouble a)
{
    if (a > 0.0) return 1.0;
    if (a < 0.0) return -1.0;
    return 0.0;
}

// "Oblique View Frustum Depth Projection and Clipping", by Eric Lengyel
void enableObliqueClipPlane(GLdouble clipPlane[4])
{
    GLdouble matrix[16];
    GLdouble q[4];

    glGetDoublev(GL_PROJECTION_MATRIX, matrix);

    q[0] = (glDoubleSign(clipPlane[0]) + matrix[8]) / matrix[0];
    q[1] = (glDoubleSign(clipPlane[1]) + matrix[9]) / matrix[5];
    q[2] = -1.0;
    q[3] = (1.0 + matrix[10]) / matrix[14];

    GLdouble c[4];
    GLdouble f = 2.0 / (clipPlane[0] * q[0] + clipPlane[1] * q[1] + clipPlane[2] * q[2] + clipPlane[3] * q[3]);

    c[0] = clipPlane[0] * f;
    c[1] = clipPlane[1] * f;
    c[2] = clipPlane[2] * f;
    c[3] = clipPlane[3] * f;

    matrix[2] = c[0];
    matrix[6] = c[1];
    matrix[10] = c[2] + 1.0;
    matrix[14] = c[3];

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(matrix);
    glMatrixMode(GL_MODELVIEW);
}

void disableObliqueClipPlane()
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void enableViewClipPlane()
{
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

    GLdouble camera[3] = { -(modelview[0] * modelview[12] + modelview[1] * modelview[13] + modelview[2] * modelview[14]),
                           -(modelview[4] * modelview[12] + modelview[5] * modelview[13] + modelview[6] * modelview[14]),
                           -(modelview[8] * modelview[12] + modelview[9] * modelview[13] + modelview[10] * modelview[14]) };

    GLdouble clipPlane[4] = { -modelview[2], -modelview[6], -modelview[10], 0 };
    GLdouble clipPoint[3] = { camera[0] - 3 * modelview[2], camera[1] - 3 * modelview[6], camera[2] - 3 * modelview[10] };

    clipPlane[3] = -(clipPoint[0] * clipPlane[0] + clipPoint[1] * clipPlane[1] + clipPoint[2] * clipPlane[2]);

    // enable cube clip plane
    glClipPlane(GL_CLIP_PLANE0 + 0, clipPlane);
    glEnable(GL_CLIP_PLANE0 + 0);

    //enableObliqueClipPlane(cubeClipPlane);
}

void disableViewClipPlane()
{
    // disable cube clip plane
    glDisable(GL_CLIP_PLANE0 + 0);

    //disableObliqueClipPlane();
}

void setColorGL(const QColor &color)
{
    glColor3ub(color.red(), color.green(), color.blue());
}

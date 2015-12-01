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
//
// taken from: Swiftless Tutorials, Game Programming and Computer Graphics Tutorials
//
// http://www.swiftless.com/tutorials/glsl/5_lighting_perpixel.html#MCPP
//
#ifndef __SHADER_H
#define __SHADER_H

#include <string>
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>

#include <boost/noncopyable.hpp>

class Shader
    : private boost::noncopyable
{
public:
    Shader(const char *vsFile, const char *fsFile);
    ~Shader();

    void bind() const;
    void unbind() const;

    GLuint id() const;

private:
    GLuint m_program;
    GLuint m_vertexShader;
    GLuint m_fragmentShader;

    void    init(const char *vsFile, const char *fsFile);
    char *  readTextFile(const char *fileName);
    void    checkShaderCompileError(GLuint shader, const char *file);
    void    checkProgramLinkError(GLuint program);
};

#endif

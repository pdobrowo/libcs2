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
#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <stdlib.h>
#include <GL/glew.h>
#include <boost/noncopyable.hpp>

/**
 * taken from: Swiftless Tutorials, Game Programming and Computer Graphics Tutorials
 *
 * http://www.swiftless.com/tutorials/glsl/5_lighting_perpixel.html#MCPP
 */

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
    bool m_loaded;
    GLuint m_program;
    GLuint m_vertexShader;
    GLuint m_fragmentShader;

    void    init(const char *vsFile, const char *fsFile);
    char *  readTextFile(const char *fileName);
    void    checkShaderCompileError(GLuint shader, const char *file);
    void    checkProgramLinkError(GLuint program);
};

#endif // SHADER_H

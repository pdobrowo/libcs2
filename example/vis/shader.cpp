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
#include "shader.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>

/**
 * taken from: Swiftless Tutorials, Game Programming and Computer Graphics Tutorials
 *
 * http://www.swiftless.com/tutorials/glsl/5_lighting_perpixel.html#MCPP
 *
 * @ fixed memleaks, reformatted
 */
Shader::Shader(const char *vsFile, const char *fsFile)
{
    init(vsFile, fsFile);
}

void Shader::init(const char *vsFile, const char *fsFile)
{
    m_vertexShader = glCreateShader(GL_VERTEX_SHADER);
    m_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    char *vertexShaderSource = readTextFile(vsFile);
    char *fragmentProgramSource = readTextFile(fsFile);

    if (vertexShaderSource == 0 || fragmentProgramSource == 0)
    {
        fprintf(stderr, "Vertex shader or fragment shader not found!\n");
        return;
    }

    const GLchar *vertexShaderLines = vertexShaderSource;
    const GLchar *fragmentProgramLines = fragmentProgramSource;

    glShaderSource(m_vertexShader, 1, &vertexShaderLines, 0);
    glShaderSource(m_fragmentShader, 1, &fragmentProgramLines, 0);

    glCompileShader(m_vertexShader);
    checkShaderCompileError(m_vertexShader, vsFile);
    glCompileShader(m_fragmentShader);
    checkShaderCompileError(m_fragmentShader, fsFile);

    m_program = glCreateProgram();
    glAttachShader(m_program, m_fragmentShader);
    glAttachShader(m_program, m_vertexShader);
    glLinkProgram(m_program);
    checkProgramLinkError(m_program);

    free(vertexShaderSource);
    free(fragmentProgramSource);
}

Shader::~Shader()
{
    glDetachShader(m_program, m_fragmentShader);
    glDetachShader(m_program, m_vertexShader);

    glDeleteShader(m_fragmentShader);
    glDeleteShader(m_vertexShader);
    glDeleteProgram(m_program);
}

GLuint Shader::id() const
{
    return m_program;
}

void Shader::bind() const
{
    glUseProgram(m_program);
}

void Shader::unbind() const
{
    glUseProgram(0);
}

char *Shader::readTextFile(const char *fileName)
{
    FILE *file = fopen(fileName, "rt");

    if (!file)
        return 0;

    fseek(file, 0, SEEK_END);
    int count = ftell(file);
    rewind(file);

    char *text = 0;

    if (count > 0)
    {
        text = static_cast<char *>(malloc(sizeof(char) * (count + 1)));
        count = fread(text, sizeof(char), count, file);
        text[count] = '\0';
    }

    fclose(file);
    return text;
}

void Shader::checkShaderCompileError(GLuint shader, const char *file)
{
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if (!success)
    {
        GLchar msg[1024] = {};
        glGetShaderInfoLog(shader, sizeof(msg), 0, msg);
        fprintf(stderr, "[%s] Shader compile error: %s\n", file, msg);
    }
}

void Shader::checkProgramLinkError(GLuint program)
{
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success)
    {
        GLchar msg[1024] = {};
        glGetProgramInfoLog(program, sizeof(msg), 0, msg);
        fprintf(stderr, "Program link error: %s\n", msg);
    }

    // validate program
    glValidateProgram(program);

    GLint status;
    glGetProgramiv(program, GL_VALIDATE_STATUS, &status);

    if (!status)
        fprintf(stderr, "Program validate error\n");
}

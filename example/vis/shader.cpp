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
// @ fixed memleaks, reformatted
//
#include "shader.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>

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

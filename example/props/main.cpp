/**
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Przemysław Dobrowolski
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
#include "cs2/vec3f.h"
#include "cs2/vec3x.h"

int main()
{
    vec3f_t a = { 1, 2, 3 };
    vec3f_t b = { 4, 5, 6 };
    vec3f_t r;

    vec3x_t u, v, w;

    vec3f_add(&r, &a, &b);

    vec3x_init(&u);
    vec3x_init(&v);
    vec3x_init(&w);

    vec3x_set_si(&u, 1, 2, 3);
    vec3x_set_si(&v, 4, 5, 6);

    vec3x_add(&w, &u, &v);

    vec3x_clear(&u);
    vec3x_clear(&v);
    vec3x_clear(&w);

    return 0;
}

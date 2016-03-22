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
#include "cs2/predtt3f.h"

void predtt3f_decomp(predttdecomp3f_t *d, const predtt3f_t *tt)
{
    preds3f_set(&d->s[0][0], &tt->k, &tt->l, &tt->a, &tt->b);
    preds3f_set(&d->s[0][1], &tt->k, &tt->l, &tt->b, &tt->c);
    preds3f_set(&d->s[0][2], &tt->k, &tt->l, &tt->c, &tt->a);
    preds3f_set(&d->s[1][0], &tt->l, &tt->m, &tt->a, &tt->b);
    preds3f_set(&d->s[1][1], &tt->l, &tt->m, &tt->b, &tt->c);
    preds3f_set(&d->s[1][2], &tt->l, &tt->m, &tt->c, &tt->a);
    preds3f_set(&d->s[2][0], &tt->m, &tt->k, &tt->a, &tt->b);
    preds3f_set(&d->s[2][1], &tt->m, &tt->k, &tt->b, &tt->c);
    preds3f_set(&d->s[2][2], &tt->m, &tt->k, &tt->c, &tt->a);
}

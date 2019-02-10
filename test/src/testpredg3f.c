/**
 * Copyright (c) 2015-2019 Przemysław Dobrowolski
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
#include "test/testpredg3f.h"

const struct cs2_predg3f_s test_predg3f_an_empty_set = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_points = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -1080.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -1000.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_y_touching_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -648.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_yz_crossed_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    0.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_z_touching_ellipsoids = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    -648.0
};

const struct cs2_predg3f_s test_predg3f_a_y_barrel = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    17.0
};

const struct cs2_predg3f_s test_predg3f_a_z_barrel = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    17.0
};

const struct cs2_predg3f_s test_predg3f_a_notched_y_barrel = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    648.0
};

const struct cs2_predg3f_s test_predg3f_a_notched_z_barrel = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    648.0
};

const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_yz_caps = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    1080.0
};

const struct cs2_predg3f_s test_predg3f_a_xy_zw_torus = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 4.0, -2.0, -4.0 },
    { -10.0, 5.0, 10.0 },
    111.0
};

const struct cs2_predg3f_s test_predg3f_a_xy_circle = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 4.0, -2.0, -4.0 },
    { -10.0, 5.0, 10.0 },
    756.0
};

const struct cs2_predg3f_s test_predg3f_a_zw_circle = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 4.0, -2.0, -4.0 },
    { -10.0, 5.0, 10.0 },
    -756.0
};

const struct cs2_predg3f_s test_predg3f_a_xz_yw_torus = {
    { -10.0, 5.0, 10.0 },
    { 4.0, -2.0, -4.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    111.0
};

const struct cs2_predg3f_s test_predg3f_a_xz_circle = {
    { -10.0, 5.0, 10.0 },
    { 4.0, -2.0, -4.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    756.0
};

const struct cs2_predg3f_s test_predg3f_a_yw_circle = {
    { -10.0, 5.0, 10.0 },
    { 4.0, -2.0, -4.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    -756.0
};

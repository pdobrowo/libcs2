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
#ifndef CS2_TESTPREDG3F_H
#define CS2_TESTPREDG3F_H

#include "cs2/predg3f.h"

/**
 * P = K x L
 * Q = A - B
 * U = K - L
 * V = A x B
 * a = ||P|| ||Q||
 * b = ||U|| ||V||
 * c = c
 *
 * Ellipsoidal:
 * K = [-3, 10, 4]
 * L = [-1, -6, -4]
 * A = [6, 2, -5]
 * B = [10, -2, -3]
 *
 * P = [-16, -16, 28]
 * Q = [-4, 4, -2]
 * U = [-2, 16, 8]
 * V = [-16, -32, -32]
 *
 * ||P|| = 36
 * ||Q|| = 6
 * ||U|| = 18
 * ||V|| = 48
 *
 * a = 216
 * b = 864
 *
 * Toroidal:
 * K = [-3, 10, 4]
 * L = [-1, -6, -4]
 * A = [4, -2, -4]
 * B = [-10, 5, 10]
 *
 * P = [-16, -16, 28]
 * Q = [14, -7, -14]
 * U = [-2, 16, 8]
 * V = [0, 0, 0]
 *
 * ||P|| = 36
 * ||Q|| = 21
 * ||U|| = 18
 * ||V|| = 0
 *
 * a = 756
 * b = 0
 */

/* common */
extern const struct cs2_predg3f_s test_predg3f_an_empty_set;

/* ellipsoidal */
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_points;
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_ellipsoids;
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_y_touching_ellipsoids;
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_yz_crossed_ellipsoids;
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_z_touching_ellipsoids;
extern const struct cs2_predg3f_s test_predg3f_a_y_barrel;
extern const struct cs2_predg3f_s test_predg3f_a_z_barrel;
extern const struct cs2_predg3f_s test_predg3f_a_notched_y_barrel;
extern const struct cs2_predg3f_s test_predg3f_a_notched_z_barrel;
extern const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_yz_caps;

/* toroidal */
extern const struct cs2_predg3f_s test_predg3f_a_xy_zw_torus;
extern const struct cs2_predg3f_s test_predg3f_a_xy_circle;
extern const struct cs2_predg3f_s test_predg3f_a_zw_circle;
extern const struct cs2_predg3f_s test_predg3f_a_xz_yw_torus;
extern const struct cs2_predg3f_s test_predg3f_a_xz_circle;
extern const struct cs2_predg3f_s test_predg3f_a_yw_circle;

#endif /* CS2_TESTPREDG3F_H */

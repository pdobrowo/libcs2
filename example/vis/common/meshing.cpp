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
#include "meshing.h"
#include <boost/scoped_array.hpp>

namespace meshing
{
namespace // anonymous
{
typedef QPair<int, int> Coord2i;

int clampCoord(int x, int m)
{
    return (x + m) % m;
}

QVector3D projectSpin(struct cs2_spin3f_s &spin)
{
    return QVector3D(spin.s12 / (1.0 - spin.s0), spin.s23 / (1.0 - spin.s0), spin.s31 / (1.0 - spin.s0));
}

double projectedDistance(const struct cs2_spin3f_s *a, const struct cs2_spin3f_s *b)
{
    struct cs2_vec3f_s pa, pb, pc;

    pa.x = a->s12 / (1.0 - a->s0);
    pa.y = a->s23 / (1.0 - a->s0);
    pa.z = a->s31 / (1.0 - a->s0);

    pb.x = b->s12 / (1.0 - b->s0);
    pb.y = b->s23 / (1.0 - b->s0);
    pb.z = b->s31 / (1.0 - b->s0);

    cs2_vec3f_sub(&pc, &pa, &pb);

    return cs2_vec3f_len(&pc);
}

void autoMeshInternal(TriangleListPtr trianglesFront, TriangleListPtr trianglesBack, struct cs2_predgparam3f_s *param, double targetRadius, int component, double minU, double maxU, double minV, double maxV, int maxSubdivisions, int subdivision)
{
    struct cs2_spin3f_s sp00, sp01, sp10, sp11;

    cs2_predgparam3f_eval(&sp00, param, minU, minV, component);
    cs2_predgparam3f_eval(&sp01, param, minU, maxV, component);
    cs2_predgparam3f_eval(&sp10, param, maxU, minV, component);
    cs2_predgparam3f_eval(&sp11, param, maxU, maxV, component);

    if (subdivision == maxSubdivisions || (projectedDistance(&sp00, &sp01) <= targetRadius && projectedDistance(&sp00, &sp10) <= targetRadius && projectedDistance(&sp00, &sp11) <= targetRadius &&
                                           projectedDistance(&sp01, &sp10) <= targetRadius && projectedDistance(&sp01, &sp11) <= targetRadius && projectedDistance(&sp10, &sp11) <= targetRadius))
    {
        QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
        QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
        QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
        QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

        QVector3D nu = QVector3D::crossProduct(v00 - v11, v01 - v11).normalized();
        QVector3D nl = QVector3D::crossProduct(v10 - v11, v00 - v11).normalized();

        Triangle tu(v00, v01, v11,
                    nu, nu, nu,
                    nu, nu, nu);

        Triangle tl(v00, v11, v10,
                    nl, nl, nl,
                    nl, nl, nl);

        if (trianglesFront)
        {
            trianglesFront->push_back(tu);
            trianglesFront->push_back(tl);
        }

        if (trianglesBack)
        {
            trianglesBack->push_back(tu.flipped());
            trianglesBack->push_back(tl.flipped());
        }
    }
    else
    {
        autoMeshInternal(trianglesFront, trianglesBack, param, targetRadius, component, minU, minU + 0.5 * (maxU - minU), minV, minV + 0.5 * (maxV - minV), maxSubdivisions, subdivision + 1);
        autoMeshInternal(trianglesFront, trianglesBack, param, targetRadius, component, minU, minU + 0.5 * (maxU - minU), minV + 0.5 * (maxV - minV), maxV, maxSubdivisions, subdivision + 1);
        autoMeshInternal(trianglesFront, trianglesBack, param, targetRadius, component, minU + 0.5 * (maxU - minU), maxU, minV, minV + 0.5 * (maxV - minV), maxSubdivisions, subdivision + 1);
        autoMeshInternal(trianglesFront, trianglesBack, param, targetRadius, component, minU + 0.5 * (maxU - minU), maxU, minV + 0.5 * (maxV - minV), maxV, maxSubdivisions, subdivision + 1);
    }
}

void simpleMeshGenPatch(TriangleListPtr trianglesFront, TriangleListPtr trianglesBack, int c, Coord2i controls[4][4], int evalCacheSize, QVector3D *evalCache)
{
    // ----
    // |\ |
    // | \|
    // ----
    //
    // [UV]
    //
    // U V -
    // |
    //
    // 00 u 01 u 02 u 03
    //   l    l    l
    // 10 u 11 u 12 u 13
    //   l    l    l
    // 20 u 21 u 22 u 23
    //   l    l    l
    // 30   31   32   33
    QVector3D vp[4][4]; // (u, v)
    QVector3D vn[3][3][2]; // (u, v, [u, l])
    QVector3D vs[2][2]; // central (u, v)

    for (int u = 0; u < 4; ++u)
        for (int v = 0; v < 4; ++v)
            vp[u][v] = evalCache[evalCacheSize * evalCacheSize * c + evalCacheSize * controls[u][v].first + controls[u][v].second];

    for (int u = 0; u < 3; ++u)
        for (int v = 0; v < 3; ++v)
            vn[u][v][0] = QVector3D::crossProduct(vp[u][v] - vp[u + 1][v + 1], vp[u][v + 1] - vp[u + 1][v + 1]).normalized();

    for (int u = 0; u < 3; ++u)
        for (int v = 0; v < 3; ++v)
            vn[u][v][1] = QVector3D::crossProduct(vp[u + 1][v] - vp[u + 1][v + 1], vp[u][v] - vp[u + 1][v + 1]).normalized();

    vs[0][0] = (vn[1][1][0] + vn[1][1][1] + vn[0][1][1] + vn[1][0][0] + vn[0][0][0] + vn[0][0][1]).normalized();
    vs[0][1] = (vn[1][2][0] + vn[1][2][1] + vn[0][2][1] + vn[1][1][0] + vn[0][1][0] + vn[0][1][1]).normalized();
    vs[1][0] = (vn[2][1][0] + vn[2][1][1] + vn[1][1][1] + vn[2][0][0] + vn[1][0][0] + vn[1][0][1]).normalized();
    vs[1][1] = (vn[2][2][0] + vn[2][2][1] + vn[1][2][1] + vn[2][1][0] + vn[1][1][0] + vn[1][1][1]).normalized();

    Triangle tu(vp[1][1], vp[1][2], vp[2][2],
                vn[1][1][0], vn[1][1][0], vn[1][1][0],
                vs[0][0], vs[0][1], vs[1][1]);

    Triangle tl(vp[1][1], vp[2][2], vp[2][1],
                vn[1][1][1], vn[1][1][1], vn[1][1][1],
                vs[0][0], vs[1][1], vs[1][0]);

    if (trianglesFront)
    {
        trianglesFront->push_back(tu);
        trianglesFront->push_back(tl);
    }

    if (trianglesBack)
    {
        trianglesBack->push_back(tu.flipped());
        trianglesBack->push_back(tl.flipped());
    }
}

} // namespace anonymous

void autoMesh(TriangleListPtr trianglesFront, TriangleListPtr trianglesBack, struct cs2_predgparam3f_s *param, double initialRadius, double targetRadius, int maxSubdivisions)
{
    int number_of_components = cs2_predgparamtype3f_components(param->t);

    for (int component = 0; component < number_of_components; ++component)
        for (double u = 0; u <= 1 - initialRadius; u += initialRadius)
            for (double v = 0; v <= 1 - initialRadius; v += initialRadius)
                autoMeshInternal(trianglesFront, trianglesBack, param, targetRadius, component, u, u + initialRadius, v, v + initialRadius, maxSubdivisions, 0);
}

void simpleMesh(TriangleListPtr trianglesFront, TriangleListPtr trianglesBack, struct cs2_predgparam3f_s *param, double radius)
{
    int numOfComponents = cs2_predgparamtype3f_components(param->t);
    Coord2i controls[4][4]; /* [u][v] */

    // calculate eval cache size
    int evalCacheSize = 0;

    for (double cs = 0.0; cs < 1.0; cs += radius)
        ++evalCacheSize;

    ++evalCacheSize;

    // compute eval cache
    boost::scoped_array<QVector3D> evalCache(new QVector3D[evalCacheSize * evalCacheSize * numOfComponents]); // (u, v, c)
    struct cs2_spin3f_s spin;
    int u, v, c;
    double pu, pv;

    for (c = 0; c < numOfComponents; ++c)
    {
        for (u = 0, pu = 0.0; pu < 1.0; pu += radius, ++u)
        {
            for (v = 0, pv = 0.0; pv < 1.0; pv += radius, ++v)
            {
                cs2_predgparam3f_eval(&spin, param, pu, pv, c);
                evalCache[evalCacheSize * evalCacheSize * c + evalCacheSize * u + v] = projectSpin(spin);
            }
        }

        for (u = 0, pu = 0.0; pu < 1.0; pu += radius, ++u)
        {
            cs2_predgparam3f_eval(&spin, param, pu, 1.0, c);
            evalCache[evalCacheSize * evalCacheSize * c + evalCacheSize * u + (evalCacheSize - 1)] = projectSpin(spin);
        }

        for (v = 0, pv = 0.0; pv < 1.0; pv += radius, ++v)
        {
            cs2_predgparam3f_eval(&spin, param, 1.0, pv, c);
            evalCache[evalCacheSize * evalCacheSize * c + evalCacheSize * (evalCacheSize - 1) + v] = projectSpin(spin);
        }

        cs2_predgparam3f_eval(&spin, param, 1.0, 1.0, c);
        evalCache[evalCacheSize * evalCacheSize * c + evalCacheSize * (evalCacheSize - 1) + evalCacheSize - 1] = projectSpin(spin);
    }

    // perform meshing
    for (int c = 0; c < numOfComponents; ++c)
    {
        for (int pu = 0; pu < evalCacheSize; ++pu)
        {
            for (int pv = 0; pv < evalCacheSize; ++pv)
            {
                for (int u = 0; u < 4; ++u)
                    for (int v = 0; v < 4; ++v)
                        controls[u][v] = Coord2i(clampCoord(pu + u - 1, evalCacheSize), clampCoord(pv + v - 1, evalCacheSize));

                simpleMeshGenPatch(trianglesFront, trianglesBack, c, controls, evalCacheSize, evalCache.get());
            }
        }
    }

//    for (int pu = 0; pu < evalCacheSize - 1; ++pu)
//    {
//        for (int u = 0; u < 4; ++u)
//            for (int v = 0; v < 4; ++v)
//                controls[u][v] = Coord2i(clampZeroUpper(pu + u - 1, evalCacheSize - 1), clampZeroUpper(evalCacheSize + v - 2, evalCacheSize - 1));

//        for (int c = 0; c < numOfComponents; ++c)
//            simpleMeshGenPatch(trianglesFront, trianglesBack, c, controls, evalCacheSize, evalCache.get());
//    }

//    for (int pv = 0; pv < evalCacheSize - 1; ++pv)
//    {
//        for (int u = 0; u < 4; ++u)
//            for (int v = 0; v < 4; ++v)
//                controls[u][v] = Coord2i(clampZeroUpper(evalCacheSize + u - 2, evalCacheSize - 1), clampZeroUpper(pv + v - 1, evalCacheSize - 1));

//        for (int c = 0; c < numOfComponents; ++c)
//            simpleMeshGenPatch(trianglesFront, trianglesBack, c, controls, evalCacheSize, evalCache.get());
//    }

//    for (int u = 0; u < 4; ++u)
//        for (int v = 0; v < 4; ++v)
//            controls[u][v] = Coord2i(clampZeroUpper(evalCacheSize + u - 2, evalCacheSize - 1), clampZeroUpper(evalCacheSize + v - 2, evalCacheSize - 1));

//    for (int c = 0; c < numOfComponents; ++c)
//        simpleMeshGenPatch(trianglesFront, trianglesBack, c, controls, evalCacheSize, evalCache.get());
}

} // namespace meshing

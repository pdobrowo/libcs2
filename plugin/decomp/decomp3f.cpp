/**
 * Copyright (c) 2015 Przemysław Dobrowolsa->i
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY a->IND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "decomp3f.h"
#include "cs2/vec3f.h"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/convex_decomposition_3.h>
#include <stdexcept>
#include <cstdlib>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;
typedef Polyhedron_3::HalfedgeDS HalfedgeDS;
typedef CGAL::Nef_polyhedron_3<Kernel, CGAL::SNC_indexed_items> Nef_polyhedron_3;
typedef Nef_polyhedron_3::Volume_const_iterator Volume_const_iterator;

template <class HDS>
class converter : public CGAL::Modifier_base<HDS>
{
public:
    converter()
    {
    }

    void operator()( HDS& hds)
    {
        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface( 3, 1, 6);
        typedef typename HDS::Vertex Vertex;
        typedef typename Vertex::Point Point;
        builder.add_vertex(Point(0, 0, 0));
        builder.add_vertex(Point(1, 0, 0));
        builder.add_vertex(Point(0, 1, 0));
        builder.begin_facet();
        builder.add_vertex_to_facet(0);
        builder.add_vertex_to_facet(1);
        builder.add_vertex_to_facet(2);
        builder.end_facet();
        builder.end_surface();
    }
};

static void decomp3f_conv(Polyhedron_3 *p, const decompmesh3f_t *m)
{
//    Polyhedron P;
//    Build_triangle<HalfedgeDS> triangle;
//    P.delegate( triangle);
//    return 0;
}

static void decomp3f_conv(decompmesh3f_t *m, const Polyhedron_3 *p)
{
//    Polyhedron P;
//    Build_triangle<HalfedgeDS> triangle;
//    P.delegate( triangle);
//    return 0;
}

static void decompmesh3f_init(decompmesh3f_t *m)
{
    m->v = 0;
    m->vs = 0;
    m->i = 0;
    m->is = 0;
}

static void decompmesh3f_clear(decompmesh3f_t *m)
{
    if (m->v)
        free(m->v);

    m->vs = 0;

    if (m->i)
        free(m->i);

    m->is = 0;
}

void decomp3f_init(decomp3f_t *d)
{
    d->t = decomptype3f_undefined;
    d->m = 0;
    d->ms = 0;
}

void decomp3f(decomp3f_t *d, const decompmesh3f_t *dm)
{
    /* clear any previous decomposition */
    decomp3f_clear(d);

    /* prepare result */
    d->t = decomptype3f_internal_error;

    try
    {
        size_t i;

        /* convert mesh to polyhedron */
        Polyhedron_3 p;
        decomp3f_conv(&p, dm);

        /* convert polyhedron to nef polyhedron */
        Nef_polyhedron_3 np(p);

        /* convex decomposition */
        CGAL::convex_decomposition_3(np);

        /* count convex parts */
        size_t n = 0;
        Volume_const_iterator vi;

        for (vi = ++np.volumes_begin(); vi != np.volumes_end(); ++vi)
            if (vi->mark())
                ++n;

        /* alloc convex parts */
        d->m = static_cast<decompmesh3f_t *>(malloc(sizeof(decompmesh3f_t) * n));

        if (!d->m)
        {
            d->t = decomptype3f_out_of_memory;
            return;
        }

        d->ms = n;

        for (i = 0; i < d->ms; ++i)
            decompmesh3f_init(d->m + i);

        /* convert convex parts */
        i = 0;

        for (vi = ++np.volumes_begin(); vi != np.volumes_end(); ++vi)
        {
            if (vi->mark())
            {
                /* convert nef polyhedron to polyhedron */
                Polyhedron_3 cp;
                np.convert_inner_shell_to_polyhedron(vi->shells_begin(), cp);

                /* convert polyhedron to mesh */
                decomp3f_conv(d->m + i, &cp);

                /* next convex part */
                ++i;
            }
        }

        /* success */
        d->t = decomptype3f_success;
    }
    catch (const std::bad_alloc &)
    {
        /* out of memory */
        d->t = decomptype3f_out_of_memory;
    }
    catch (const std::exception &)
    {
        /* unknown exception */
        d->t = decomptype3f_internal_error;
    }
}

void decomp3f_clear(decomp3f_t *d)
{
    if (d->m)
    {
        for (size_t i = 0; i < d->ms; ++i)
            decompmesh3f_clear(d->m + i);

        free(d->m);
    }

    d->ms = 0;
    d->t = decomptype3f_undefined;
}

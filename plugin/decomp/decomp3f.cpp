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
class Polyhedron_builder
    : public CGAL::Modifier_base<HDS>
{
    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;

public:
    Polyhedron_builder(const decompmesh3f_t *m)
        : m_m(m)
    {
    }

    void operator()(HDS &hds)
    {
        size_t i, j;

        /* polyhedron builder */
        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);

        /* create surface */
        builder.begin_surface(m_m->vs, m_m->fs);

        /* create vertex list */
        for (i = 0; i < m_m->vs; ++i)
            builder.add_vertex(conv(m_m->v + i));

        /* create facets */
        for (i = 0; i < m_m->fs; ++i)
        {
            builder.begin_facet();

            for (j = 0; j < m_m->f[i].is; ++j)
                builder.add_vertex_to_facet(m_m->f[i].i[j]);

            builder.end_facet();
        }

        /* finish surface */
        builder.end_surface();
    }

private:
    const decompmesh3f_t *m_m;

    Point conv(const vec3f_t *v)
    {
        return Point(v->x, v->y, v->z);
    }
};

static void decomp3f_conv(Polyhedron_3 *p, const decompmesh3f_t *m)
{
    Polyhedron_builder<HalfedgeDS> c(m);
    p->delegate(c);
}

static void decomp3f_conv(decompmesh3f_t *m, const Polyhedron_3 *p)
{
    size_t vs = p->size_of_vertices();
    size_t fs = p->size_of_facets();

    m->v = static_cast<vec3f_t *>(malloc(sizeof(vec3f_t) * vs));

    if (!m->v)
        return; // todo: handle

    m->f = static_cast<decompface3f_t *>(malloc(sizeof(decompface3f_t) * fs));

    if (!m->f)
    {
        free(m->v);
        m->v = 0;
        return; // todo: handle
    }

    m->vs = vs;
    m->fs = fs;

    /* convert */
    size_t i = 0;

    for (Polyhedron_3::Vertex_const_iterator vi = p->vertices_begin(); vi != p->vertices_end(); ++vi)
    {
        m->v[i].x = CGAL::to_double(vi->point().x());
        m->v[i].y = CGAL::to_double(vi->point().y());
        m->v[i].z = CGAL::to_double(vi->point().z());
        ++i;
    }

    i = 0;

    for (Polyhedron_3::Facet_const_iterator fi = p->facets_begin(); fi != p->facets_end(); ++fi)
    {
            Polyhedron_3::Halfedge_around_facet_const_circulator hc = fi->facet_begin();
            size_t is = CGAL::circulator_size(hc);

            m->f[i].i = static_cast<size_t *>(malloc(sizeof(size_t) * is));

            if (!m->f[i].i)
                return; // todo: handle

            m->f[i].is = is;

            size_t j = 0;

            do
            {
                m->f[i].i[j] = std::distance(p->vertices_begin(), hc->vertex());
                ++j;
            }
            while (++hc != fi->facet_begin());
    }
}

static void decompmesh3f_init(decompmesh3f_t *m)
{
    m->v = 0;
    m->vs = 0;
    m->f = 0;
    m->fs = 0;
}

static void decompmesh3f_clear(decompmesh3f_t *m)
{
    if (m->v)
        free(m->v);

    m->vs = 0;

    if (m->f)
        free(m->f);

    m->fs = 0;
}

void decomp3f_init(decomp3f_t *d)
{
    d->t = decomptype3f_undefined;
    d->m = 0;
    d->ms = 0;
}

void decomp3f_make(decomp3f_t *d, const decompmesh3f_t *dm)
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

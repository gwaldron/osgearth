/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <limits.h>

#include <osgEarth/Tessellator>

using namespace osgEarth;


#define LC "[Tessellator] "

/***************************************************/

namespace
{

// Borrowed from osgUtil/DelaunayTriangulator.cpp
// Compute the circumcircle of a triangle (only x and y coordinates are used),
// return (Cx, Cy, r^2)
inline osg::Vec3 compute_circumcircle(
    const osg::Vec3 &a,
    const osg::Vec3 &b,
    const osg::Vec3 &c)
{
    float D =
        (a.x() - c.x()) * (b.y() - c.y()) -
        (b.x() - c.x()) * (a.y() - c.y());

    float cx, cy, r2;

    if(D==0.0)
    {
        // (Nearly) degenerate condition - either two of the points are equal (which we discount)
        // or the three points are colinear. In this case we just determine the average of
        // the three points as the centre for correctness, but squirt out a zero radius.
        // This method will produce a triangulation with zero area, so we have to check later
        cx = (a.x()+b.x()+c.x())/3.0;
        cy = (a.y()+b.y()+c.y())/3.0;
        r2 = 0.0;
    }
    else
    {
        cx =
        (((a.x() - c.x()) * (a.x() + c.x()) +
        (a.y() - c.y()) * (a.y() + c.y())) / 2 * (b.y() - c.y()) -
        ((b.x() - c.x()) * (b.x() + c.x()) +
        (b.y() - c.y()) * (b.y() + c.y())) / 2 * (a.y() - c.y())) / D;

        cy =
        (((b.x() - c.x()) * (b.x() + c.x()) +
        (b.y() - c.y()) * (b.y() + c.y())) / 2 * (a.x() - c.x()) -
        ((a.x() - c.x()) * (a.x() + c.x()) +
        (a.y() - c.y()) * (a.y() + c.y())) / 2 * (b.x() - c.x())) / D;

      //  r2 = (c.x() - cx) * (c.x() - cx) + (c.y() - cy) * (c.y() - cy);
        // the return r square is compared with r*r many times in an inner loop
        // so for efficiency use the inefficient sqrt once rather than 30* multiplies later.
        r2 = sqrt((c.x() - cx) * (c.x() - cx) + (c.y() - cy) * (c.y() - cy));
    }
    return osg::Vec3(cx, cy, r2);
}

// Test whether a point (only the x and y coordinates are used) lies inside
// a circle; the circle is passed as a vector: (Cx, Cy, r).

inline bool point_in_circle(const osg::Vec3 &point, const osg::Vec3 &circle)
{
    float r2 =
        (point.x() - circle.x()) * (point.x() - circle.x()) +
        (point.y() - circle.y()) * (point.y() - circle.y());
    return r2 <= circle.z()*circle.z();
//    return r2 <= circle.z();
}

int checkCCW(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double v = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    return (v == 0.0 ? 0 : (v > 0.0 ? 1 : -1));
}

//bool point_in_tri(const osg::Vec3 &p, const osg::Vec3 &t1, const osg::Vec3 &t2, const osg::Vec3 &t3)
bool point_in_tri(double xp, double yp, double x1, double y1, double x2, double y2, double x3, double y3)
{
  int t1 = checkCCW(xp, yp, x1, y1, x2, y2);
  int t2 = checkCCW(xp, yp, x2, y2, x3, y3);
  int t3 = checkCCW(xp, yp, x3, y3, x1, y1);

  return t1 == t2 && t2 == t3;
}

struct TriIndices
{
    unsigned int a;
    unsigned int b;
    unsigned int c;

    TriIndices(unsigned int p1, unsigned int p2, unsigned int p3)
      : a(p1), b(p2), c(p3)
    {
      //nop
    }
};

typedef std::vector<TriIndices> TriList;

}

bool
Tessellator::tessellateGeometry(osg::Geometry &geom)
{
    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());

    if (!vertices || vertices->empty() || geom.getPrimitiveSetList().empty()) return false;

    // copy the original primitive set list
    osg::Geometry::PrimitiveSetList originalPrimitives = geom.getPrimitiveSetList();

    // clear the primitive sets
    unsigned int nprimsetoriginal= geom.getNumPrimitiveSets();
    if (nprimsetoriginal) geom.removePrimitiveSet(0, nprimsetoriginal);

    bool success = true;
    for (unsigned int i=0; i < originalPrimitives.size(); i++)
    {
        osg::ref_ptr<osg::PrimitiveSet> primitive = originalPrimitives[i].get();

        if (primitive->getMode()==osg::PrimitiveSet::POLYGON || primitive->getMode()==osg::PrimitiveSet::LINE_LOOP)
        {
            if (primitive->getType()==osg::PrimitiveSet::DrawArrayLengthsPrimitiveType)
            {
                osg::DrawArrayLengths* drawArrayLengths = static_cast<osg::DrawArrayLengths*>(primitive.get());
                unsigned int first = drawArrayLengths->getFirst();
                for(osg::DrawArrayLengths::iterator itr=drawArrayLengths->begin();
                    itr!=drawArrayLengths->end();
                    ++itr)
                {
                    unsigned int last = first + *itr;
                    osg::PrimitiveSet* newPrimitive = tessellatePrimitive(first, last, vertices);
                    if (newPrimitive)
                    {
                        geom.addPrimitiveSet(newPrimitive);
                    }
                    else
                    {
                        // tessellation failed, add old primitive set back
                        geom.addPrimitiveSet(primitive);
                        success = false;
                    }

                    first = last;
                }
            }
            else
            {
                if (primitive->getNumIndices()>=3)
                {
                    osg::PrimitiveSet* newPrimitive = tessellatePrimitive(primitive.get(), vertices);
                    if (newPrimitive)
                    {
                        geom.addPrimitiveSet(newPrimitive);
                    }
                    else
                    {
                        // tessellation failed, add old primitive set back
                        geom.addPrimitiveSet(primitive);
                        success = false;
                    }
                }
            }
        }
        else
        {
            //
            // TODO: handle more primitive modes
            //
        }
    }

    return success;
}


osg::PrimitiveSet*
Tessellator::tessellatePrimitive(osg::PrimitiveSet* primitive, osg::Vec3Array* vertices)
{
    //
    //TODO: Hnadle more primitive types
    //

    switch(primitive->getType())
    {
    case(osg::PrimitiveSet::DrawArraysPrimitiveType):
        {
            osg::DrawArrays* drawArray = static_cast<osg::DrawArrays*>(primitive);
            unsigned int first = drawArray->getFirst();
            unsigned int last = first+drawArray->getCount();
            return tessellatePrimitive(first, last, vertices);
         }
    default:
        OE_NOTICE << LC << "Primitive type " << primitive->getType()<< " not handled" << std::endl;
        break;
    }

    return 0L;
}

osg::PrimitiveSet*
Tessellator::tessellatePrimitive(unsigned int first, unsigned int last, osg::Vec3Array* vertices)
{
    std::vector<unsigned int> activeVerts;
    activeVerts.reserve( last-first+1 );
    for (unsigned int i=first; i < last; i++)
    {
        activeVerts.push_back(i);
    }

    TriList tris;
    tris.reserve( activeVerts.size() );

    bool success = true;
    unsigned int cursor = 0;
    unsigned int cursor_start = 0;
    unsigned int tradCursor = UINT_MAX;
    while (activeVerts.size() > 3)
    {
        if (isConvex(*vertices, activeVerts, cursor))
        {
            bool tradEar = tradCursor != UINT_MAX;
            if (isEar(*vertices, activeVerts, cursor, tradEar))
            {
                unsigned int prev = cursor == 0 ? activeVerts.size() - 1 : cursor - 1;
                unsigned int next = cursor == activeVerts.size() - 1 ? 0 : cursor + 1;

                tris.push_back(TriIndices(activeVerts[prev], activeVerts[cursor], activeVerts[next]));
              
                activeVerts.erase(activeVerts.begin() + cursor);
                if (cursor >= activeVerts.size())
                    cursor = 0;

                cursor_start = cursor;
                tradCursor = UINT_MAX;

                continue;
            }
            
            if (tradEar && tradCursor == UINT_MAX)
            {
                tradCursor = cursor;
            }
        }

        cursor++;
        if (cursor >= activeVerts.size())
            cursor = 0;

        if (cursor == cursor_start)
        {
            if (tradCursor != UINT_MAX)
            {
                // No ear was found with circumcircle test, use first traditional ear found

                cursor = tradCursor;

                unsigned int prev = cursor == 0 ? activeVerts.size() - 1 : cursor - 1;
                unsigned int next = cursor == activeVerts.size() - 1 ? 0 : cursor + 1;

                tris.push_back(TriIndices(activeVerts[prev], activeVerts[cursor], activeVerts[next]));
                  
                activeVerts.erase(activeVerts.begin() + cursor);
                if (cursor >= activeVerts.size())
                    cursor = 0;

                cursor_start = cursor;
                tradCursor = UINT_MAX;

                continue;
            }
            else
            {
                success = false;
                break;
            }
        }
    }

    if (success)
    {
        if (activeVerts.size() == 3)
        {
            // add last tri
            tris.push_back(TriIndices(activeVerts[0], activeVerts[1], activeVerts[2]));
        }

        osg::DrawElementsUInt* triElements = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
        triElements->reserve( tris.size() * 3 );
        for (TriList::const_iterator it = tris.begin(); it != tris.end(); ++it)
        {
            triElements->push_back(it->a);
            triElements->push_back(it->b);
            triElements->push_back(it->c);
        }

        return triElements;
    }
    else
    {
        //TODO: handle?
        OE_DEBUG << LC << "Tessellation failed!" << std::endl;
    }

    return 0L;
}


bool
Tessellator::isConvex(const osg::Vec3Array &vertices, const std::vector<unsigned int> &activeVerts, unsigned int cursor)
{
    unsigned int prev = cursor == 0 ? activeVerts.size() - 1 : cursor - 1;
    unsigned int next = cursor == activeVerts.size() - 1 ? 0 : cursor + 1;

    unsigned int a = activeVerts[prev];
    unsigned int b = activeVerts[cursor];
    unsigned int c = activeVerts[next];

    osg::Vec3d dataA;
    dataA._v[0] = vertices[a][0];
    dataA._v[1] = vertices[a][1];
    dataA._v[2] = vertices[a][2];

    osg::Vec3d dataB;
    dataB._v[0] = vertices[b][0];
    dataB._v[1] = vertices[b][1];
    dataB._v[2] = vertices[b][2];

    osg::Vec3d dataC;
    dataC._v[0] = vertices[c][0];
    dataC._v[1] = vertices[c][1];
    dataC._v[2] = vertices[c][2];

    //http://www.gamedev.net/topic/542870-determine-which-side-of-a-line-a-point-is/#entry4500667
    //(Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax)

    return (dataB.x() - dataA.x()) * (dataC.y() - dataA.y()) - (dataB.y() - dataA.y()) * (dataC.x() - dataA.x()) > 0.0;
}

bool
Tessellator::isEar(const osg::Vec3Array &vertices, const std::vector<unsigned int> &activeVerts, unsigned int cursor, bool &tradEar)
{
    unsigned int prev = cursor == 0 ? activeVerts.size() - 1 : cursor - 1;
    unsigned int next = cursor == activeVerts.size() - 1 ? 0 : cursor + 1;

    osg::Vec3d cc(compute_circumcircle(vertices[activeVerts[prev]], vertices[activeVerts[cursor]], vertices[activeVerts[next]]));

    unsigned int nextNext = next == activeVerts.size() - 1 ? 0 : next + 1;

		// Check every point not part of the ear
    bool circEar = true;
		while( nextNext != prev )
		{
        unsigned int p = activeVerts[nextNext];

        if (circEar && point_in_circle(vertices[p], cc))
        {
            circEar = false;

            if (tradEar)
              return false;
        }

        if (!tradEar &&
            point_in_tri(vertices[p].x(), vertices[p].y(),
                         vertices[activeVerts[prev]].x(), vertices[activeVerts[prev]].y(),
                         vertices[activeVerts[cursor]].x(), vertices[activeVerts[cursor]].y(),
                         vertices[activeVerts[next]].x(), vertices[activeVerts[next]].y()))
			  {
            return false;
			  }

			  nextNext = nextNext == activeVerts.size() - 1 ? 0 : nextNext + 1;
		}

    tradEar = true;

		return circEar;
}
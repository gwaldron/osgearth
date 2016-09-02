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

#include "BoundaryUtil"
#include "VertexCollectionVisitor"
#include <algorithm>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/TriangleIndexFunctor>
#include <osg/ComputeBoundsVisitor>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/SpatialReference>
#include <osgDB/WriteFile>
#include <map>
#include <vector>
#include <set>

/* Comparator used to sort osg::Vec3d's first by x and then by y */
bool presortCompare (osg::Vec3d i, osg::Vec3d j)
{
  if (i.x() == j.x())
    return i.y() < j.y();

  return i.x() < j.x();
}

double BoundaryUtil::_tolerance = 0.005;

void
BoundaryUtil::setTolerance(double value)
{
    _tolerance = value;
}

/* Use the vertices of the given node to calculate a boundary via the
 * findHull() method.
 */
osg::Vec3dArray* BoundaryUtil::getBoundary(osg::Node* modelNode, bool geocentric, bool convexHull)
{
  if (!modelNode)
    return 0;

  if ( convexHull )
  {
    VertexCollectionVisitor v(geocentric);
    modelNode->accept(v);

    osg::ref_ptr<osg::Vec3dArray> verts = v.getVertices();
    verts = findHull(*verts);

    osg::EllipsoidModel em;
    for( osg::Vec3dArray::iterator i = verts->begin(); i != verts->end(); ++i )
    {
      em.convertLatLongHeightToXYZ( osg::DegreesToRadians(i->y()), osg::DegreesToRadians(i->x()), i->z(),
        i->x(), i->y(), i->z() );      
    }

    return verts.release();
  }
  else
  {
    return findMeshBoundary( modelNode, geocentric );
  }
}

/* Finds the convex hull for the given points using the Andrew's monotone
 * chian algorithm. Returns an ordered set of points defining the hull
 *
 * Implementation based on chainHull_2D() method from 
 * softSurfer (www.softsurfer.com)
 */
osg::Vec3dArray* BoundaryUtil::findHull(osg::Vec3dArray& points)
{
  if (points.size() == 0)
    return 0;

  // the output array hull will be used as the stack
  osg::Vec3dArray* hull = new osg::Vec3dArray(points.size());

  // Presort the points as required by the algorithm
  osg::ref_ptr<osg::Vec3dArray> sorted = hullPresortPoints(points);
  
  int bot=0, top=(-1);  // indices for bottom and top of the stack
  int i;                // array scan index
  int n = sorted->size();

  // Get the indices of points with min x-coord and min|max y-coord
  int minmin = 0, minmax;
  double xmin = (*sorted)[0].x();
  for (i=1; i<n; i++)
    if ((*sorted)[i].x() != xmin) break;
  minmax = i-1;

  //if the points at minmin and minmax have the same value, shift minmin
  //to ignore the duplicate points
  if ((*sorted)[minmin] == (*sorted)[minmax])
    minmin = minmax;

  // degenerate case: all x-coords == xmin
  if (minmax == n-1)
  {       
    (*hull)[++top] = (*sorted)[minmin];
    if ((*sorted)[minmax].y() != (*sorted)[minmin].y()) // a nontrivial segment
      (*hull)[++top] = (*sorted)[minmax];
    (*hull)[++top] = (*sorted)[minmin];           // add polygon endpoint

    hull->resize(top + 1);
    return hull;
  }

  // Get the indices of points with max x-coord and min|max y-coord
  int maxmin, maxmax = n-1;
  double xmax = (*sorted)[n-1].x();
  for (i=n-2; i>=0; i--)
    if ((*sorted)[i].x() != xmax) break;
  maxmin = i+1;

  // Compute the lower hull on the stack
  (*hull)[++top] = (*sorted)[minmin];  // push minmin point onto stack
  i = minmax;
  while (++i <= maxmin)
  {
    // the lower line joins (*sorted)[minmin] with (*sorted)[maxmin]

    // if multiple points have the same x/y values, go with the lowest z value
    if ((*hull)[top].x() == (*sorted)[i].x() && (*hull)[top].y() == (*sorted)[i].y())
    {
      if ((*sorted)[i].z() < (*hull)[top].z())
        (*hull)[top].z() = (*sorted)[i].z();

      continue;
    }

    if (isLeft((*sorted)[minmin], (*sorted)[maxmin], (*sorted)[i]) > 0 && i < maxmin)
      continue;  // ignore (*sorted)[i] above the lower line.  NOTE: Differs from original CH algorithm in that it keeps collinear points

    while (top > 0)  // there are at least 2 points on the stack
    {
      // test if (*sorted)[i] is left of or on the line at the stack top. NOTE: Differs from original CH algorithm in that it keeps collinear points
      if (isLeft((*hull)[top-1], (*hull)[top], (*sorted)[i]) >= 0)
        break;  // (*sorted)[i] is a new hull vertex
      else
        top--;  // pop top point off stack
    }
    (*hull)[++top] = (*sorted)[i];  // push (*sorted)[i] onto stack
  }

  if (maxmax != maxmin)  // if distinct xmax points
  {
    // Push all points between maxmin and maxmax onto stack. NOTE: Differs from original CH algorithm in that it keeps collinear points
    while (i <= maxmax)
    {
      if ((*hull)[top].x() == (*sorted)[i].x() && (*hull)[top].y() == (*hull)[top].y())
      {
        if ((*sorted)[i].z() < (*hull)[top].z())
          (*hull)[top].z() = (*sorted)[i].z();
      }
      else
      {
        (*hull)[++top] = (*sorted)[i];
      }

      i++;
    }
  }

  // Next, compute the upper hull on the stack H above the bottom hull

  bot = top;  // the bottom point of the upper hull stack
  i = maxmin;
  while (--i >= minmax)
  {
    // the upper line joins (*sorted)[maxmax] with (*sorted)[minmax]

    // if multiple points have the same x/y values, go with the lowest z value
    if ((*hull)[top].x() == (*sorted)[i].x() && (*hull)[top].y() == (*sorted)[i].y())
    {
      if ((*sorted)[i].z() < (*hull)[top].z())
        (*hull)[top].z() = (*sorted)[i].z();

      continue;
    }

    if (isLeft((*sorted)[maxmax], (*sorted)[minmax], (*sorted)[i]) > 0 && i > minmax)
      continue;  // ignore (*sorted)[i] below the upper line. NOTE: Differs from original CH algorithm in that it keeps collinear points

    while (top > bot)  // at least 2 points on the upper stack
    {
      // test if (*sorted)[i] is left of or on the line at the stack top. NOTE: Differs from original CH algorithm in that it keeps collinear points
      if (isLeft((*hull)[top-1], (*hull)[top], (*sorted)[i]) >= 0)
        break;   // (*sorted)[i] is a new hull vertex
      else
        top--;   // pop top point off stack
    }
    (*hull)[++top] = (*sorted)[i];  // push (*sorted)[i] onto stack
  }

  // If minmax and minmin are the same, remove the duplicate point
  if (minmax == minmin)
  {
    top--;
  }
  else
  {
    // Push all points between minmax and minmin onto stack. NOTE: Differs from original CH algorithm in that it keeps collinear points
    while (i > minmin)
    {
      if ((*hull)[top].x() == (*sorted)[i].x() && (*hull)[top].y() == (*hull)[top].y())
      {
        if ((*sorted)[i].z() < (*hull)[top].z())
          (*hull)[top].z() = (*sorted)[i].z();
      }
      else
      {
        (*hull)[++top] = (*sorted)[i];
      }

      i--;
    }
  }

  hull->resize(top + 1);
  return hull;
}

/* Returns an array containing the points sorted first by x and then by y */
osg::Vec3dArray* BoundaryUtil::hullPresortPoints(osg::Vec3dArray& points)
{
  osg::Vec3dArray* sorted = new osg::Vec3dArray(points.begin(), points.end());
  std::sort(sorted->begin(), sorted->end(), presortCompare);

  return sorted;
}

//---------------------------------------------------------------------------

namespace
{
    // custom comparator for VertexSet.
    struct VertexLess
    {
        bool operator()(const osg::Vec3d& lhs, const osg::Vec3d& rhs) const
        {
            double dx = lhs.x() - rhs.x();
            if ( dx < 0.0 && dx < -BoundaryUtil::getTolerance() ) return true;
            if ( dx > 0.0 && dx >  BoundaryUtil::getTolerance() ) return false;

            double dy = lhs.y() - rhs.y();
            return (dy < 0.0 && dy < -BoundaryUtil::getTolerance());
        }
    };

    typedef std::set<osg::Vec3d, VertexLess> VertexSet;
    typedef VertexSet::iterator Index;

    // custom comparator for Index so we can use it at a std::map key
    struct IndexLess : public std::less<Index> {
      bool operator()(const Index& lhs, const Index& rhs ) const {
        return (*lhs) < (*rhs);
      }
    };
    
    typedef std::set<Index, IndexLess>            IndexSet;
    typedef std::map<Index, IndexSet, IndexLess>  EdgeMap;

    // Stores the noded topology of a model with unique verticies and edge definitions.
    // The verticies are stored rotated into the XY plane so that we can properly find
    // the "bottom-most" vert and walk the boundary.
    struct TopologyGraph
    {
        TopologyGraph()
          : _minY( _verts.end() ), _totalVerts(0), _srs(0L) { }

        unsigned     _totalVerts;  // total number of verts encountered
        VertexSet    _vertsWorld;  // 
        VertexSet    _verts;       // set of unique verts in the topology (rotated into XY plane)
        EdgeMap      _edgeMap;     // maps each vert to all the verts with which it shares an edge
        Index        _minY;        // points to the vert with the minimum Y coordinate (in XY plane)
        osg::Matrixd _world2plane; // matrix that transforms into a localized XY plane
        const osgEarth::SpatialReference* _srs;
    };

    typedef std::map<unsigned,Index> UniqueMap;

    // A TriangleIndexFunctor that traverses a stream of triangles and builds a
    // topology graph from their points and edges.
    struct TopologyBuilder
    {
        TopologyGraph*  _topology;     // topology to which to append point and edge data
        osg::Vec3Array* _vertexList;   // source vertex list
        osg::Matrixd    _local2world;  // transforms source verts into world coordinates
        UniqueMap       _uniqueMap;    // prevents duplicates

        void operator()( unsigned v0, unsigned v1, unsigned v2 )
        {
            Index i0 = add( v0 );
            Index i1 = add( v1 );
            Index i2 = add( v2 );

            // add to the edge list for each of these verts
            if ( i0 != i1 ) _topology->_edgeMap[i0].insert( i1 );
            if ( i0 != i2 ) _topology->_edgeMap[i0].insert( i2 );
            if ( i1 != i0 ) _topology->_edgeMap[i1].insert( i0 );
            if ( i1 != i2 ) _topology->_edgeMap[i1].insert( i2 );
            if ( i2 != i0 ) _topology->_edgeMap[i2].insert( i0 );
            if ( i2 != i1 ) _topology->_edgeMap[i2].insert( i1 );
        }

        Index add( unsigned v )
        {
            // first see if we already added the vert at this index.
            UniqueMap::iterator i = _uniqueMap.find( v );
            if ( i == _uniqueMap.end() )
            {
                // no, so transform it into world coordinates, and rotate it into the XY plane
                osg::Vec3d vert = (*_vertexList)[v];
                osg::Vec3d world = vert * _local2world;
                osg::Vec3d plane = world;

                if ( _topology->_srs )
                {
                    const osgEarth::SpatialReference* ecef = _topology->_srs->getECEF();
                    ecef->transform(world, _topology->_srs, plane);
                }
                else
                {
                    plane = world * _topology->_world2plane;
                }

                // insert it into the unique vert list
                std::pair<VertexSet::iterator,bool> f = _topology->_verts.insert( plane );
                if ( f.second ) // insert succedded
                {
                    // this is a new location, so check it to see if it is the new "southernmost" point:
                    if ( _topology->_minY == _topology->_verts.end() || plane.y() < _topology->_minY->y() )
                    {
                        _topology->_minY = f.first;
                    }
                }

                // store in the uniqueness map so we don't process the same index again
                _uniqueMap[ v ] = f.first;

                // return the index of the vert.
                return f.first;
            }
            else
            {
                return i->second;
            }
        }
    };

    // Visits a scene graph and builds a topology graph from the verts and edges
    // found within.
    struct BuildTopologyVisitor : public osg::NodeVisitor
    {
        BuildTopologyVisitor( TopologyGraph& topology )
            : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
              _topology( topology )
        {
            //nop
        }

        // track local transforms so we can build a topology in world coords
        void apply( osg::Transform& xform )
        {
            osg::Matrix matrix;
            if ( !_matrixStack.empty() ) matrix = _matrixStack.back();
            xform.computeLocalToWorldMatrix( matrix, this );
            _matrixStack.push_back( matrix );
            traverse( xform );
            _matrixStack.pop_back();
        }

        // add the contents of a geode to the topology
        void apply( osg::Geode& geode )
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Drawable* drawable = geode.getDrawable( i );
                if ( drawable->asGeometry() )
                {
                    apply( drawable->asGeometry() );
                }
            }
        }

        // add the contents of a Geometry to the topology
        void apply( osg::Geometry* geometry )
        {
            osg::Vec3Array* vertexList = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
            if ( vertexList )
            {
                osg::TriangleIndexFunctor<TopologyBuilder> builder;
                builder._topology = &_topology;
                builder._vertexList = vertexList;
                if ( !_matrixStack.empty() )
                    builder._local2world = _matrixStack.back();
                _topology._totalVerts += vertexList->size();
                geometry->accept( builder );
            }
        }

        std::vector<osg::Matrixd> _matrixStack;
        TopologyGraph&            _topology;
    };

    void dumpPointCloud(TopologyGraph& t)
    {
        osg::Vec3Array* v = new osg::Vec3Array();
        osg::DrawElementsUInt* lines = new osg::DrawElementsUInt(GL_LINES);
        unsigned index = 0;
        unsigned minyindex = 0;
        std::map<Index,unsigned, IndexLess> order;
        for(Index i = t._verts.begin(); i != t._verts.end(); ++i, ++index)
        {
            v->push_back( *i );
            if ( i == t._minY )
                minyindex = index;
            order[i] = index;
        }
        index = 0;
        for(Index i = t._verts.begin(); i != t._verts.end(); ++i, ++index)
        {
            IndexSet& edges = t._edgeMap[i];
            for(IndexSet::iterator j=edges.begin(); j!=edges.end(); ++j)
            {
                lines->push_back(order[i]);
                lines->push_back(order[*j]);
            }
        }
        osg::Geometry* g = new osg::Geometry();
        g->setVertexArray(v);
        g->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, v->size()));
        g->addPrimitiveSet(lines);
        g->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3));
        osg::Geode* n = new osg::Geode();
        n->addDrawable(g);
        osg::Geometry* g2 = new osg::Geometry();
        g2->setVertexArray(v);
        g2->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, minyindex, 1));
        g2->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(10));
        n->addDrawable(g2);
        osgDB::writeNodeFile(*n, "mesh.osg");            
        n->unref();
    }
}

//------------------------------------------------------------------------

osg::Vec3dArray*
BoundaryUtil::findMeshBoundary( osg::Node* node, bool geocentric )
{
    // the normal defines the XY plane in which to search for a boundary
    osg::Vec3d normal(0,0,1);
    osg::Vec3d center;

    if ( geocentric )
    {
        // define the XY plane based on the normal to the center of the dataset:
        osg::BoundingSphere bs = node->getBound();
        center = bs.center();
        normal = center;
        normal.normalize();
        OE_DEBUG << "Normal = " << normal.x() << ", " << normal.y() << ", " << normal.z() << std::endl;
    }

    osg::ref_ptr<osg::Vec3dArray> _result = new osg::Vec3dArray();

    // first build a topology graph from the node.
    TopologyGraph topology;

    // set up a transform that will localize geometry into an XY plane
    if ( geocentric )
    {
        topology._world2plane.makeRotate(normal, osg::Vec3d(0,0,1));
        topology._world2plane.preMultTranslate(-center);

        // if this is set, use mercator projection instead of a simple geolocation
        //topology._srs = osgEarth::SpatialReference::get("spherical-mercator");
    }

    // build the topology
    BuildTopologyVisitor buildTopoVisitor(topology);
    node->accept( buildTopoVisitor );

    OE_DEBUG << "Found " << topology._verts.size() << " unique verts" << std::endl;
    //dumpPointCloud(topology);

    // starting with the minimum-Y vertex (which is guaranteed to be in the boundary)
    // traverse the outside of the point set. Do this by sorting all the edges by
    // their angle relative to the vector to the previous point. The vector with the
    // smallest angle represents the edge connecting the current point to the next
    // boundary point. Walk the edge until we return to the beginning.
    
    Index vptr      = topology._minY;
    Index vptr_prev = topology._verts.end();

    IndexSet visited;

    while( true )
    {
        // store this vertex in the result set:
        _result->push_back( *vptr );

        if ( _result->size() == 56 )
        {
            int asd=0;
        }

        // pull up the next 2D vertex (XY plane):
        osg::Vec2d vert ( vptr->x(), vptr->y() );

        // construct the "base" vector that points from the previous 
        // point to the current point; or to -X in the initial case
        osg::Vec2d base;
        if ( vptr_prev == topology._verts.end() )
            base.set( -1, 0 );
        else
            base = vert - osg::Vec2d( vptr_prev->x(), vptr_prev->y() );
            
        // pull up the edge set for this vertex:
        IndexSet& edges = topology._edgeMap[vptr];

        // find the edge with the minimun delta angle to the base vector
        double bestScore = DBL_MAX;
        Index  bestEdge  = topology._verts.end();
        
        OE_DEBUG << "VERTEX (" << 
            vptr->x() << ", " << vptr->y() << ", " << vptr->z() 
            << ") has " << edges.size() << " edges..."
            << std::endl;

        for( IndexSet::iterator e = edges.begin(); e != edges.end(); ++e )
        {
            // don't go back from whence we just came
            if ( *e == vptr_prev )
                continue;

            // never return to a vert we've already visited
            if ( visited.find(*e) != visited.end() )
                continue;

            // calculate the angle between the base vector and the current edge:
            osg::Vec2d edgeVert( (*e)->x(), (*e)->y() );
            osg::Vec2d edge = edgeVert - vert;

            base.normalize();
            edge.normalize();
            double cross = base.x()*edge.y() - base.y()*edge.x();
            double dot   = base * edge;
            double score = dot;

            if ( cross < 0.0 )
            {
                double diff = 2.0-(score+1.0);
                score = 1.0 + diff;
            }

            OE_DEBUG << "   check: " << (*e)->x() << ", " << (*e)->y() << ", " << (*e)->z() << std::endl;
            OE_DEBUG << "   base = " << base.x() << ", " << base.y() << std::endl;
            OE_DEBUG << "   edge = " << edge.x() << ", " << edge.y() << std::endl;
            OE_DEBUG << "   crs = " << cross << ", dot = " << dot << ", score = " << score << std::endl;
            
            if ( score < bestScore )
            {
                bestScore = score;
                bestEdge = *e;
            }
        }

        if ( bestEdge == topology._verts.end() )
        {
            // this will probably never happen
            osg::notify(osg::WARN) << "Illegal state - reached a dead end!" << std::endl;
            break;
        }

        // store the previous:
        vptr_prev = vptr;

        // follow the chosen edge around the outside of the geometry:
        OE_DEBUG << "   BEST SCORE = " << bestScore << std::endl;
        vptr = bestEdge;

        // record this vert so we don't visit it again.
        visited.insert( vptr );

        // once we make it all the way around, we're done:
        if ( vptr == topology._minY )
            break;
    }

    // un-rotate the results from the XY plane back to their original frame:
    if ( topology._srs )
    {
        const osgEarth::SpatialReference* ecef = topology._srs->getECEF();
        topology._srs->transform(_result->asVector(), ecef);
    }
    else
    {
        osg::Matrix plane2world;
        plane2world.invert( topology._world2plane );
        for( osg::Vec3dArray::iterator i = _result->begin(); i != _result->end(); ++i )
            (*i) = (*i) * plane2world;
    }

    return _result.release();
}

//------------------------------------------------------------------------

bool
BoundaryUtil::simpleBoundaryTest(const osg::Vec3dArray& boundary)
{
  osg::ref_ptr<osgEarth::Symbology::Polygon> boundsPoly = new osgEarth::Symbology::Polygon();
  for (int i=0; i < (int)boundary.size(); i++)
    boundsPoly->push_back(boundary[i]);

  osgEarth::Bounds boundsBounds = boundsPoly->getBounds();

  osg::ref_ptr<osgEarth::Symbology::Polygon> outterPoly = new osgEarth::Symbology::Polygon();
  outterPoly->push_back(osg::Vec3d(boundsBounds.xMin() - 10.0, boundsBounds.yMin() - 10.0, boundsBounds.zMin()));
  outterPoly->push_back(osg::Vec3d(boundsBounds.xMax() + 10.0, boundsBounds.yMin() - 10.0, boundsBounds.zMin()));
  outterPoly->push_back(osg::Vec3d(boundsBounds.xMax() + 10.0, boundsBounds.yMax() + 10.0, boundsBounds.zMin()));
  outterPoly->push_back(osg::Vec3d(boundsBounds.xMin() - 10.0, boundsBounds.yMax() + 10.0, boundsBounds.zMin()));

  osg::ref_ptr<osgEarth::Symbology::Geometry> outPoly;
  return outterPoly->difference(boundsPoly, outPoly);
}

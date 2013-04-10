/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "BoundaryUtil"
#include "VertexCollectionVisitor"
#include <algorithm>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/TriangleIndexFunctor>
#include <osgEarthSymbology/Geometry>
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
    typedef std::set<osg::Vec3d> VertexSet; 
    typedef VertexSet::iterator  Index;

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
          : _minY( _verts.end() ) { }

        VertexSet _verts;    // set of unique verts in the topology (rotated into XY plane)
        osg::Quat _rot;      // rotates a geocentric vert into the XY plane
        EdgeMap   _edgeMap;  // maps each vert to all the verts with which it shares an edge
        Index     _minY;     // points to the vert with the minimum Y coordinate (in XY plane)
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
            _topology->_edgeMap[i0].insert( i1 );
            _topology->_edgeMap[i0].insert( i2 );
            _topology->_edgeMap[i1].insert( i0 );
            _topology->_edgeMap[i1].insert( i2 );
            _topology->_edgeMap[i2].insert( i0 );
            _topology->_edgeMap[i2].insert( i1 );
        }

        Index add( unsigned v )
        {
            // first see if we already added this vert:
            UniqueMap::iterator i = _uniqueMap.find( v );
            if ( i == _uniqueMap.end() )
            {
              // no, so transform it into world coordinates, and rotate it into the XY plane
              osg::Vec3d spVert = (*_vertexList)[v];
              osg::Vec3d local = _topology->_rot * (spVert * _local2world);

              // insert it into the unique vert list
              std::pair<VertexSet::iterator,bool> f = _topology->_verts.insert( local );
              if ( f.second )
              {
                // this is a new location, so check it to see if it is the new "lowest" point:
                if ( _topology->_minY == _topology->_verts.end() || local.y() < _topology->_minY->y() )
                  _topology->_minY = f.first;

                // store in the uniqueness map to prevent duplication
                _uniqueMap[ v ] = f.first;
              }
             
              // return the index of the vert.
              return f.first;
            }
            else
            {
              // return the index of the vert.
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
                osg::TriangleIndexFunctor<TopologyBuilder> _builder;
                _builder._topology = &_topology;
                _builder._vertexList = vertexList;
                if ( !_matrixStack.empty() )
                  _builder._local2world = _matrixStack.back();
                geometry->accept( _builder );
            }
        }

        std::vector<osg::Matrixd> _matrixStack;
        TopologyGraph&            _topology;
    };
}

//------------------------------------------------------------------------

osg::Vec3dArray*
BoundaryUtil::findMeshBoundary( osg::Node* node, bool geocentric )
{
    // the normal defines the XY plane in which to search for a boundary
    osg::Vec3d normal(0,0,1);

    if ( geocentric )
    {
        // define the XY plane based on the normal to the center of the dataset:
        osg::BoundingSphere bs = node->getBound();
        normal = bs.center();
        normal.normalize();
    }

    osg::ref_ptr<osg::Vec3dArray> _result = new osg::Vec3dArray();

    // first build a topology graph from the node.
    TopologyGraph topology;

    // set up a quat that will rotate geometry into our XY plane
    if ( normal != osg::Vec3(0,0,1) )
        topology._rot.makeRotate( normal, osg::Vec3d(0,0,1) );

    // build the topology
    BuildTopologyVisitor buildTopoVisitor(topology);
    node->accept( buildTopoVisitor );

    // starting with the minimum-Y vertex (which is guaranteed to be in the boundary)
    // traverse the outside of the point set. Do this by sorting all the edges by
    // their angle relative to the vector to the previous point. The vector with the
    // smallest angle represents the edge connecting the current point to the next
    // boundary point. Walk the edge until we return to the beginning.
    
    Index vptr      = topology._minY;
    Index vptr_prev = topology._verts.end();

    while( true )
    {
        // store this vertex in the result set:
        _result->push_back( *vptr );

        // pull up the next 2D vertex (XY plane):
        osg::Vec2d vert( vptr->x(), vptr->y() );

        // construct the "base" vector that points from the current point back
        // to the previous point; or to -X in the initial case
        osg::Vec2d base;
        if ( vptr_prev == topology._verts.end() )
            base.set( -1, 0 );
        else
            base = osg::Vec2d( vptr_prev->x(), vptr_prev->y() ) - vert;
            
        // pull up the edge set for this vertex:
        IndexSet& edges = topology._edgeMap[vptr];

        // find the edge with the minimun delta angle to the base vector
        double minAngle = DBL_MAX;
        Index  minEdge  = topology._verts.end();

        for( IndexSet::iterator e = edges.begin(); e != edges.end(); ++e )
        {
            // don't go back from whence we just came
            if ( *e == vptr_prev )
              continue;

            // calculate the angle between the base vector and the current edge:
            osg::Vec2d edgeVert( (*e)->x(), (*e)->y() );
            osg::Vec2d edge = edgeVert - vert;

            double baseAngle = atan2(base.y(), base.x());
            double edgeAngle = atan2(edge.y(), edge.x());

            double outsideAngle = baseAngle - edgeAngle;

            // normalize it to [0..360)
            if ( outsideAngle < 0.0 )
              outsideAngle += 2.0*osg::PI;

            // see it is qualifies as the new minimum angle
            if ( outsideAngle < minAngle )
            {
                minAngle = outsideAngle;
                minEdge = *e;
            }
        }

        if ( minEdge == topology._verts.end() )
        {
            // this will probably never happen
            osg::notify(osg::WARN) << "Illegal state - bailing" << std::endl;
            return 0L;
        }

        vptr_prev = vptr;

        // follow the chosen edge around the outside of the geometry:
        vptr = minEdge;

        // once we make it all the way around, we're done:
        if ( vptr == topology._minY )
            break;
    }

    // un-rotate the results from the XY plane back to their original frame:
    osg::Quat invRot = topology._rot.inverse();
    for( osg::Vec3dArray::iterator i = _result->begin(); i != _result->end(); ++i )
    {
      (*i) = invRot * (*i);
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

/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarthUtil/TopologyGraph>

using namespace osgEarth::Util;

#include <osg/Point>
#include <osg/TriangleIndexFunctor>
#include <osgDB/WriteFile>

#define LC "[TopologyGraph] "


TopologyGraph::TopologyGraph() :
_maxGraphID(0u)
{
    //nop
}

unsigned
TopologyGraph::getNumBoundaries() const
{
    return _maxGraphID;
}

void
TopologyGraph::createBoundary(unsigned graphNum, TopologyGraph::IndexVector& output) const
{
    // nothing to do - bail
    if (_verts.empty() || graphNum+1 > _maxGraphID)
        return;

    // graph ID is one more than the graph number passed in:
    unsigned graphID = graphNum + 1u;

    // Find the starting point (vertex with minimum Y) for this graph ID.
    // By the nature of disconnected graphs, that start point is all we need
    // to ensure we are walking a single connected mesh.
    Index vstart = _verts.end();
    for (VertexSet::const_iterator vert = _verts.begin(); vert != _verts.end(); ++vert)
    {
        if (vert->_graphID == graphID) 
        {
            if (vstart == _verts.end() || vert->y() < vstart->y())
            {
                vstart = vert;
            }
        }
    }

    // couldn't find a start point - bail (should never happen)
    if (vstart == _verts.end())
        return;

    // starting with the minimum-Y vertex (which is guaranteed to be in the boundary)
    // traverse the outside of the point set. Do this by sorting all the edges by
    // their angle relative to the vector from the previous point. The "leftest" turn
    // represents the edge connecting the current point to the next boundary point.
    // Thusly we walk the boundary counterclockwise until we return to the start point.
    Index vptr = vstart;
    Index vptr_prev = _verts.end();

    IndexSet visited;

    while( true )
    {
        // store this vertex in the result set:
        output.push_back( vptr );

        // pull up the next 2D vertex (XY plane):
        osg::Vec2d vert ( vptr->x(), vptr->y() );

        // construct the "base" vector that points from the previous 
        // point to the current point; or to +X in the initial case
        osg::Vec2d base;
        if ( vptr_prev == _verts.end() )
        {
            base.set(1, 0);
        }
        else
        {
            base = vert - osg::Vec2d( vptr_prev->x(), vptr_prev->y() );
            base.normalize();
        }
            
        // pull up the edge set for this vertex:
        EdgeMap::const_iterator ei = _edgeMap.find(vptr);
        if (ei == _edgeMap.end())
            continue; // should be impossible

        const IndexSet& edges = ei->second;

        // find the edge with the minimum delta angle to the base vector
        double bestScore = -DBL_MAX;
        Index  bestEdge  = _verts.end();
        
        //OE_NOTICE << "VERTEX (" << 
        //    vptr->x() << ", " << vptr->y() << ", "
        //    << ") has " << edges.size() << " edges..."
        //    << std::endl;

        unsigned possibleEdges = 0u;

        for( IndexSet::iterator e = edges.begin(); e != edges.end(); ++e )
        {
            // don't go back from whence we just came
            if ( *e == vptr_prev )
                continue;

            // never return to a vert we've already visited
            if ( visited.find(*e) != visited.end() )
                continue;

            ++possibleEdges;

            // calculate the angle between the base vector and the current edge:
            osg::Vec2d edgeVert( (*e)->x(), (*e)->y() );
            osg::Vec2d edge = edgeVert - vert;
            edge.normalize();

            double cross = base.x()*edge.y() - base.y()*edge.x();
            double dot   = base * edge;

            double score;
            if (cross <= 0.0)
                score = 1.0-dot; // [0..2]
            else
                score = dot-1.0; // [-2..0]

            //OE_NOTICE << "   check: " << (*e)->x() << ", " << (*e)->y() << std::endl;
            //OE_NOTICE << "   base = " << base.x() << ", " << base.y() << std::endl;
            //OE_NOTICE << "   edge = " << edge.x() << ", " << edge.y() << std::endl;
            //OE_NOTICE << "   crs = " << cross << ", dot = " << dot << ", score = " << score << std::endl;
            
            if (score > bestScore)
            {
                bestScore = score;
                bestEdge = *e;
            }
        }

        if ( bestEdge == _verts.end() )
        {
            // this should never happen
            // but sometimes does anyway
            OE_WARN << LC << getName() << " - Illegal state - reached a dead end during boundary detection. Vertex (" 
                << vptr->x() << ", " << vptr->y() << ") has " << possibleEdges << " possible edges.\n"
                << std::endl;
            break;
        }

        // store the previous:
        vptr_prev = vptr;

        // follow the chosen edge around the outside of the geometry:
        //OE_DEBUG << "   BEST SCORE = " << bestScore << std::endl;
        vptr = bestEdge;

        // record this vert so we don't visit it again.
        visited.insert( vptr );

        // once we make it all the way around, we're done:
        if ( vptr == vstart )
            break;
    }
}

void
TopologyGraph::dumpBoundary(const IndexVector& boundary, const std::string& filename)
{
    if (boundary.empty())
        return;
    
    osg::Vec3Array* v = new osg::Vec3Array();

    for (IndexVector::const_iterator i = boundary.begin(); i != boundary.end(); ++i)
    {
        const Index& index = *i;
        v->push_back(osg::Vec3(index->x(), index->y(), 0));
    }

    osg::ref_ptr<osg::Geometry> g = new osg::Geometry();
    g->setVertexArray(v);
    g->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, 0, v->size()));
    g->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, v->size()));
    g->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3));
    osgDB::writeNodeFile(*(g.get()), filename);
}

TopologyBuilder::TopologyBuilder()
{
    //nop
}

void
TopologyBuilder::operator()(unsigned v0, unsigned v1, unsigned v2)
{
    // Add three verts to the graph. Any of them may already exist
    // in the graph. If so, make sure the existing graph IDs get
    // properly propagated.
    TopologyGraph::Index i0 = add(v0);
    TopologyGraph::Index i1 = add(v1);
    TopologyGraph::Index i2 = add(v2);

    if (i0->_graphID == 0u && i1->_graphID == 0u && i2->_graphID == 0u)
    {
        unsigned newGraphID = ++_graph->_maxGraphID;
        i0->_graphID = i1->_graphID = i2->_graphID = newGraphID;
    }
    else
    {
        if (i0->_graphID != 0u)
        {
            assignAndPropagate(i1, i0->_graphID);
            assignAndPropagate(i2, i0->_graphID);
        }
        else if (i1->_graphID != 0u)
        {
            assignAndPropagate(i0, i1->_graphID);
            assignAndPropagate(i2, i1->_graphID);
        }
        else
        {
            assignAndPropagate(i0, i2->_graphID);
            assignAndPropagate(i1, i2->_graphID);
        }
    }

    // add to the edge list for each of these verts
    if (i0 != i1) _graph->_edgeMap[i0].insert(i1);
    if (i0 != i2) _graph->_edgeMap[i0].insert(i2);
    if (i1 != i0) _graph->_edgeMap[i1].insert(i0);
    if (i1 != i2) _graph->_edgeMap[i1].insert(i2);
    if (i2 != i0) _graph->_edgeMap[i2].insert(i0);
    if (i2 != i1) _graph->_edgeMap[i2].insert(i1);
}

TopologyGraph::Index
TopologyBuilder::add(unsigned v)
{
    // first see if we already added the vert at this index.
    UniqueMap::iterator i = _uniqueMap.find(v);
    if (i == _uniqueMap.end())
    {
        TopologyGraph::Vertex vertex(_verts, v);
        std::pair<TopologyGraph::Index, bool> f = _graph->_verts.insert(vertex);
        return f.first;
    }
    else
    {
        return i->second;
    }
}

void
TopologyBuilder::assignAndPropagate(TopologyGraph::Index& vertex, unsigned graphID)
{
    // already set, so no need to continue propagating
    if (vertex->_graphID == graphID)
        return;

    // assign
    vertex->_graphID = graphID;

    // propagate along all edges
    TopologyGraph::EdgeMap::iterator edges = _graph->_edgeMap.find(vertex);
    if (edges != _graph->_edgeMap.end())
    {
        TopologyGraph::IndexSet& endPoints = edges->second;
        for (TopologyGraph::IndexSet::iterator endPoint = endPoints.begin();
            endPoint != endPoints.end();
            ++endPoint)
        {
            TopologyGraph::Index vertex = *endPoint;
            assignAndPropagate(vertex, graphID);
        }
    }
}

TopologyGraph*
TopologyBuilder::create(const osg::Vec3Array* verts, const osg::PrimitiveSet* elems, const std::string& name)
{
    osg::ref_ptr<TopologyGraph> graph = new TopologyGraph();
    graph->setName(name);

    osg::TriangleIndexFunctor<TopologyBuilder> builder;
    builder.setVertexArray(verts->getNumElements(), static_cast<const osg::Vec3*>(verts->getDataPointer()));
    builder._verts = verts;
    builder._graph = graph.get();
    elems->accept(builder);

    return graph.release();
}

BuildTopologyVisitor::BuildTopologyVisitor(TopologyGraph& graph) :
osg::NodeVisitor(),
_graph(graph)
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);
}

void
BuildTopologyVisitor::apply(osg::Transform& xform)
{
    osg::Matrix matrix;
    if (!_matrixStack.empty()) matrix = _matrixStack.back();
    xform.computeLocalToWorldMatrix(matrix, this);
    _matrixStack.push_back(matrix);
    traverse(xform);
    _matrixStack.pop_back();
}

void
BuildTopologyVisitor::apply(osg::Drawable& drawable)
{
    osg::Geometry* geom = drawable.asGeometry();
    if (geom)
    {
        osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
        if (verts)
        {
            apply(&drawable, verts);
        }
    }
}

void
BuildTopologyVisitor::apply(osg::Drawable* drawable, osg::Vec3Array* verts)
{
    osg::TriangleIndexFunctor<TopologyBuilder> builder;
    builder._graph = &_graph;
    builder._drawable = drawable;
    builder._verts = verts;
    //if (!_matrixStack.empty())
    //    builder._local2world = _matrixStack.back();
    drawable->accept(builder);
}

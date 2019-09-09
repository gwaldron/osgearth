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

#ifndef OSGEARTHUTIL_TOPOLOGY_GRAPH_H
#define OSGEARTHUTIL_TOPOLOGY_GRAPH_H 1

#include <osgEarthUtil/Common>
#include <osgEarth/SpatialReference>
#include <osg/NodeVisitor>
#include <osg/Vec3d>
#include <osg/Geometry>
#include <vector>
#include <map>
#include <set>

namespace osgEarth { namespace Util
{
#define TOPOLOGY_TOLERANCE 0.0

    //! Stores the noded topology of a model with unique vertices and edge definitions.
    //! The vertices are stored rotated into the XY plane so that we can properly find
    //! the "bottom-most" vert and walk the boundary.
    class OSGEARTHUTIL_EXPORT TopologyGraph : public osg::Object
    {
    public:

        //! Represents a single vertex in the topology graph
        struct Vertex
        {
            Vertex(const Vertex& rhs) : _verts(rhs._verts), _index(rhs._index), _graphID(0u) { }

            Vertex(const osg::Vec3Array* verts, unsigned index) : _verts(verts), _index(index), _graphID(0u) { }

            const osg::Vec3& vertex() const { return (*_verts)[_index]; }

            unsigned index() const { return _index; }

            float x() const { return (*_verts)[_index].x(); }
            float y() const { return (*_verts)[_index].y(); }

            bool operator < (const Vertex& rhs) const {
                //if (_verts < rhs._verts) return true;
                //if (_verts > rhs._verts) return false;
                //return _index < rhs._index;
                if (x() < rhs.x()) return true;
                if (x() > rhs.x()) return false;
                return y() < rhs.y();
            }

            const osg::Vec3Array* _verts;   //! vertex array from which this vertex originated
            unsigned _index;                //! index into the source vertex array
            mutable unsigned _graphID;      //! which graph does this vertex belong to (in the event of multiple graphs)
        };

        //! One or more Vertex objects
        typedef std::set<Vertex> VertexSet;

        //! Iterator into a VertexSet.
        typedef VertexSet::iterator Index;

        //! Custom comparator for Index so we can use it at a std::map key
        struct IndexLess : public std::less<Index> {
            bool operator()(const Index& lhs, const Index& rhs) const {
                return (*lhs) < (*rhs);
            }
        };

        //! Sortable set of indices
        typedef std::set<Index, IndexLess> IndexSet;

        //! Maps a Vertex Index to a collection of Vertex indices (edges)
        typedef std::map<Index, IndexSet, IndexLess> EdgeMap;

        //! Vector of Vertex Indexes
        typedef std::vector<Index> IndexVector;

    public:

        META_Object(osgEarthUtil, TopologyGraph);

        //! CTOR
        TopologyGraph();

        //! How many disconnected graphs are there in this topology?
        unsigned getNumBoundaries() const;

        //! Creates the boundary of the nth disconnected graph
        void createBoundary(unsigned boundaryNum, IndexVector& output) const;

    public:
        void addTriangle(const osg::Vec3Array* verts, unsigned v0, unsigned v1, unsigned v2);

        Index add(const osg::Vec3Array* verts, unsigned v);

        void assignAndPropagate(TopologyGraph::Index& vertex, unsigned graphID);

    public:
        unsigned     _totalVerts;  // total number of verts encountered
        VertexSet    _verts;       // set of unique verts in the topology (rotated into XY plane)
        EdgeMap      _edgeMap;     // maps each vert to all the verts with which it shares an edge
        unsigned     _maxGraphID;  // maximum graph id from builder (1=one graph)

        friend class TopologyBuilder;
        friend class BuildTopologyVisitor;

        void dumpBoundary(const IndexVector& boundary, const std::string& filename);

    protected:
        // no copy ctor
        TopologyGraph(const TopologyGraph& rhs, const osg::CopyOp& copy) { }

        virtual ~TopologyGraph() { }

        unsigned recomputeSubgraphs();
    };

    typedef std::vector< osg::ref_ptr<TopologyGraph> > TopologyGraphVector;

    // A TriangleIndexFunctor that traverses a stream of triangles and builds a
    // topology graph from their points and edges.
    class OSGEARTHUTIL_EXPORT TopologyBuilder
    {
    public:
        //! CTOR
        TopologyBuilder();

        //! Creates a new topology graph from a single set of triangles.
        //! @param[in ] verts Vertex array input
        //! @param[in ] elems Elements defining triangle set
        //! @param[in ] name  Optional name for error/warning output
        //! @return     New TopologyGraph object
        static TopologyGraph* create(
            const osg::Vec3Array*    verts,
            const osg::PrimitiveSet* elems,
            const std::string&       name = std::string());

        typedef std::map<unsigned, TopologyGraph::Index> UniqueMap;

        TopologyGraph* _graph;           // topology to which to append point and edge data
        const osg::Drawable* _drawable;  // source geometry
        const osg::Vec3Array* _verts;    // source vertex list
        UniqueMap _uniqueMap;            // prevents duplicates

        // entry point for the TriangleIndexFunctor
        void operator()( unsigned v0, unsigned v1, unsigned v2 );

        TopologyGraph::Index add( unsigned v );

        void assignAndPropagate(TopologyGraph::Index& vertex, unsigned graphID);

        friend class TopologyBuilderVisitor;
    };


    // Visits a scene graph and builds a topology graph from the verts and edges
    // found within.
    class OSGEARTHUTIL_EXPORT BuildTopologyVisitor : public osg::NodeVisitor
    {
    public:
        BuildTopologyVisitor( TopologyGraph& topology );

        // track local transforms so we can build a topology in world coords
        void apply( osg::Transform& xform );

        // add the contents of a Geometry to the topology
        void apply( osg::Drawable& drawable );

        // ???
        void apply( osg::Drawable* drawable, osg::Vec3Array* verts );

        std::vector<osg::Matrixd> _matrixStack;
        TopologyGraph& _graph;
    };

} }

#endif // OSGEARTHUTIL_TOPOLOGY_GRAPH_H

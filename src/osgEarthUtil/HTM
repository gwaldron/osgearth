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

#ifndef OSGEARTH_UTIL_HTM_H
#define OSGEARTH_UTIL_HTM_H 1

#include <osgEarthUtil/Common>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Polytope>
#include <vector>
#include <osgEarth/optional>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;

    struct HTMSettings
    {
        unsigned _maxObjectsPerCell;
        optional<float> _rangeFactor;
        optional<float> _maxRange;
        float _minCellSize;
        float _maxCellSize;
        bool _storeObjectsInLeavesOnly;
        int _debugCount;
        int _debugFrame;
        bool _debugGeom;
    };

    /**
     * Hierarchical Triangular Mesh group - for geocentric maps only
     * http://www.geog.ucsb.edu/~hu/papers/spatialIndex.pdf
     *
     * An osg::Group that automatically organizes its contents spatially
     * in order to improve culling performance.
     */
    class OSGEARTHUTIL_EXPORT HTMGroup : public osg::Group
    {
    public:
        HTMGroup();

        //! Sets the maximum number of objects that can live in a single cell.
        void setMaximumObjectsPerCell(unsigned value) { _settings._maxObjectsPerCell = value; }
        float getMaximumObjectsPerCell() const { return _settings._maxObjectsPerCell; }

        //! Minimum size (bounding radius*2) of an HTM cell.
        void setMinimumCellSize(double value) { _settings._minCellSize = value; }
        double getMinimumCellSize() const { return _settings._minCellSize; }

        //! Maximum size (bounding radius*2) of an HTM cell that can contain objects.
        void setMaximumCellSize(double value) { _settings._maxCellSize = value; }
        double getMaximumCellSize() const { return _settings._maxCellSize; }

        //! Cells become visible at (cell bounding radius * range factor).
        //! If this is set, the value in setRange is ignored.
        void setRangeFactor(float value) { _settings._rangeFactor = value; }

        //! Range at which objects become visible. This overrides range factor if set.
        void setMaxRange(float value) { _settings._maxRange = value; }

        //! If true, only store objects in the leaf nodes (defaults to false)
        void setStoreObjectsInLeavesOnly(bool value) { _settings._storeObjectsInLeavesOnly = value; }

        //! Enable debugging geometry
        void setDebug(bool value) { _settings._debugGeom = value; }
        bool getDebug() const { return _settings._debugGeom; }

    public: // osg::Group

        /** Add a node to the group. */
        virtual bool addChild(osg::Node* child);

        /** Add a node to the group. Ignores the "index". */
        virtual bool insertChild(unsigned index, osg::Node* child);


    public: // osg::Group (internal)

        /** These methods are derived from Group but are NOOPs for the HTMGroup. */
        virtual bool removeChildren(unsigned pos, unsigned numChildrenToRemove);
        virtual bool replaceChild(osg::Node* origChild, osg::Node* newChild);
        virtual bool setChild(unsigned index, osg::Node* node);

    protected:
        virtual ~HTMGroup() { }

        bool insert(osg::Node* node);

        void reinitialize();

        HTMSettings _settings;
    };


    /**
     * Internal index cell for the HTMGroup (do not use directly).
     */
    class HTMNode : public osg::Group
    {
    public:
        HTMNode(HTMSettings& settings,
                const osg::Vec3d& v0, const osg::Vec3d& v1, const osg::Vec3d& v2,
                const std::string& id = "");
        
        bool contains(const osg::Vec3d& p) const {
            return _tri.contains(p);
        }

        void insert(osg::Node* node);

    public:
        void traverse(osg::NodeVisitor& nv);

    protected:
        virtual ~HTMNode() { }

        void split();

        // test whether the node's triangle lies entirely within a frustum
        bool entirelyWithin(const osg::Polytope& tope) const;
        
        // test whether the node's triangle intersects a frustum
        bool intersects(const osg::Polytope& tope) const;

    private:

        struct PolytopeDP : public osg::Polytope
        {
            bool contains(const osg::Vec3d& p) const;
            bool containsAnyOf(const std::vector<osg::Vec3d>& p) const;
        };

        struct Triangle
        {
            std::vector<osg::Vec3d> _v;
            PolytopeDP              _tope;

            void set(const osg::Vec3d& v0, const osg::Vec3d& v1, const osg::Vec3d& v2);

            void getMidpoints(osg::Vec3d* w) const;

            bool contains(const osg::Vec3d& p) const {
                return _tope.contains(p);
            }
        };


        Triangle _tri;
        bool     _isLeaf;
        HTMSettings& _settings;
        osg::ref_ptr<osg::Node> _debug;
    };

} } // namesapce osgEarth::Util


#endif // OSGEARTH_UTIL_HTM_H

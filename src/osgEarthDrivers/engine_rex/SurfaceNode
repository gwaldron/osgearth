/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#ifndef OSGEARTH_DRIVERS_REX_SURFACE_NODE
#define OSGEARTH_DRIVERS_REX_SURFACE_NODE 1

#include "Common"
#include "GeometryPool"
#include "RenderBindings"
#include "TileDrawable"
#include "TileRenderModel"

#include <osgEarth/MapInfo>
#include <osgEarth/Horizon>
#include <osg/MatrixTransform>
#include <osg/BoundingBox>
#include <osg/Drawable>
#include <osg/Geode>
#include <osgText/Text>
#include <osgUtil/CullVisitor>
#include <OpenThreads/Atomic>
#include <vector>

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    using namespace osgEarth;

    /**
     * Like cluster culling
     */
    struct HorizonTileCuller
    {
        osg::Vec3d  _points[4];
        osg::ref_ptr<Horizon> _horizon;

        void set(const SpatialReference* srs, const osg::Matrix& local2world, const osg::BoundingBox& bbox);

        bool isVisible(const osg::Vec3d& from) const;
    };


    /**
     * SurfaceNode holds the geometry and transform information
     * for one terrain tile surface.
     */
    class SurfaceNode : public osg::MatrixTransform
    {
    public:
        SurfaceNode(
            const TileKey&        tilekey,
            const MapInfo&        mapinfo,
            const RenderBindings& bindings,
            TileDrawable*         drawable);

        void setElevationRaster(const osg::Image* raster, const osg::Matrixf& scaleBias);
        const osg::Image* getElevationRaster() const;
        const osg::Matrixf& getElevationMatrix() const;

        const osg::BoundingBox& getAlignedBoundingBox() const;
        
        TileDrawable* getDrawable() const { return _drawable.get(); }

        inline bool isVisibleFrom(const osg::Vec3& viewpoint) const {
            return _horizonCuller.isVisible(viewpoint);
        }
        
        // A box can have 4 children. 
        // Returns true if any child box intersects the sphere of radius centered around point
        inline bool anyChildBoxIntersectsSphere(const osg::Vec3& point, float radiusSquared) {
            for(int c=0; c<4; ++c) {
                for(int j=0; j<8; ++j) {
                    if ( (_childrenCorners[c][j]-point).length2() < radiusSquared )
                        return true;
                }
            }
            return false;
        }

        bool anyChildBoxWithinRange(float range, osg::NodeVisitor& nv) const {
            for(int c=0; c<4; ++c) {
                for(int j=0; j<8; ++j) {
                    if (nv.getDistanceToViewPoint(_childrenCorners[c][j], true) < range)
                        return true;
                }
            }
            return false;
        }

        void setDebugText(const std::string& strText);

        osg::BoundingSphere computeBound() const;

        osg::Node* getDebugNode() const { return _debugNode.get(); }

        void setLastFramePassedCull(unsigned fn);

        unsigned getLastFramePassedCull() const { return _lastFramePassedCull; }

        float getPixelSizeOnScreen(osg::CullStack* cull) const;

    protected:
        virtual ~SurfaceNode() { }

        TileKey                     _tileKey;
        osg::ref_ptr<TileDrawable>  _drawable;
        osg::ref_ptr<osg::Node>     _debugNode;
        osg::ref_ptr<osgText::Text> _debugText;
        static const bool           _enableDebugNodes;
        OpenThreads::Atomic         _lastFramePassedCull;
        HorizonTileCuller           _horizonCuller;        

        typedef osg::Vec3 VectorPoints[8];


        void addDebugNode(const osg::BoundingBox& box);
        void removeDebugNode(void);

        VectorPoints _worldCorners;

        typedef VectorPoints (ChildrenCorners) [4];
        ChildrenCorners _childrenCorners;
    };

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_DRIVERS_REX_SURFACE_NODE

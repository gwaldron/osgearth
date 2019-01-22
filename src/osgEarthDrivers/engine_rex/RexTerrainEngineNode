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
#ifndef OSGEARTH_DRIVERS_REX_TERRAIN_ENGINE_ENGINE_NODE_H
#define OSGEARTH_DRIVERS_REX_TERRAIN_ENGINE_ENGINE_NODE_H 1

#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/Map>
#include <osgEarth/Revisioning>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Containers>
#include <osgEarth/ResourceReleaser>
#include <osgEarth/TileRasterizer>

#include "RexTerrainEngineOptions"
#include "EngineContext"
#include "TileNodeRegistry"
#include "RenderBindings"
#include "GeometryPool"
#include "Loader"
#include "Unloader"
#include "SelectionInfo"
#include "SurfaceNode"
#include "TileDrawable"
#include "TerrainCuller"

#include <list>
#include <map>
#include <vector>

using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    class RexTerrainEngineNode : public TerrainEngineNode
    {
    public:
        META_Node(osgEarth, RexTerrainEngineNode);

        RexTerrainEngineNode();

    protected:
        virtual ~RexTerrainEngineNode();

    public: // TerrainEngineNode public

        //! for standalone tile creation outside of a terrain
        osg::Node* createTile(const TileKey& key);

        //! Create a standalone tile from a tile model (experimental)
        osg::Node* createTile(
            const TerrainTileModel* model,
            int createTileFlags,
            unsigned referenceLOD);
        
        //! Forces regeneration of tiles in the given region.
        void invalidateRegion(
            const GeoExtent& extent,
            unsigned         minLevel,
            unsigned         maxLevel);

        //! Get the stateset used to render the terrain surface.
        osg::StateSet* getSurfaceStateSet();

        //! Unique identifier of this engine instance
        UID getUID() const { return _uid; }

        //! Terrain options object
        const TerrainOptions& getTerrainOptions() const { return _terrainOptions; }

    public: // osg::Node

        void traverse(osg::NodeVisitor& nv);

        osg::BoundingSphere computeBound() const;

        void resizeGLObjectBuffers(unsigned maxSize);

        void releaseGLObjects(osg::State* state) const;

    public: // MapCallback adapter functions

        void onMapModelChanged( const MapModelChange& change ); // not virtual!

        //! Access to the asychronous data loader
        Loader* getLoader() const { return _loader.get(); }

    protected: // TerrainEngineNode protected

        virtual void setMap(const Map* map, const TerrainOptions& options);

        virtual void updateTextureCombining() { updateState(); }
        
        virtual void dirtyTerrain();

        virtual void dirtyState();

    private:
        void init();
        void syncMapModel();

        //! Reloads all the tiles in the terrain due to a data model change
        void refresh(bool force =false);

        //! Various methods that trigger when the Map model changes.
        void addLayer(Layer* layer);
        void addElevationLayer( ElevationLayer* layer );
        void addTileLayer( Layer* layer );
        void removeImageLayer( ImageLayer* layerRemoved );
        void removeElevationLayer( ElevationLayer* layerRemoved );
        void toggleElevationLayer( ElevationLayer* layer );
        void moveElevationLayer( ElevationLayer* layerMoved );
        
        //! refresh the statesets of the terrain and the imagelayer tile surface
        void updateState(); 

        //! one-time allocation of render units for the terrain
        void setupRenderBindings();
        
        //! Adds a Layer to the cachedLayerExtents vector.
        void cacheLayerExtentInMapSRS(Layer* layer); 

        //! computes the heightfield sample size required to match the vertices at the highest
        //! level of detail in rex if a tile key was requested at the given level of detail.
        unsigned int computeSampleSize(unsigned int levelOfDetail);

    private:
        RexTerrainEngineOptions _terrainOptions;

        UID _uid;
        bool _batchUpdateInProgress;
        bool _refreshRequired;
        bool _stateUpdateRequired;

        // extents of each layer, in the Map's SRS. UID = vector index (for speed)
        LayerExtentVector _cachedLayerExtents; 

        // node registry is shared across all threads.
        osg::ref_ptr<TileNodeRegistry> _liveTiles; // tiles in the scene graph.
        osg::ref_ptr<ResourceReleaser> _releaser;
     
        EngineContext* getEngineContext() const { return _engineContext.get(); }
        osg::ref_ptr< EngineContext > _engineContext;
        friend class EngineContext;

        RenderBindings _renderBindings;
        osg::ref_ptr<GeometryPool> _geometryPool;
        osg::ref_ptr<LoaderGroup>  _loader;
        osg::ref_ptr<UnloaderGroup> _unloader;
        TileRasterizer* _rasterizer;
        
        osg::ref_ptr<osg::Group> _terrain;
        bool _morphingSupported;

        bool _renderModelUpdateRequired;

        RexTerrainEngineNode( const RexTerrainEngineNode& rhs, const osg::CopyOp& op =osg::CopyOp::DEEP_COPY_ALL ) { }

        SelectionInfo _selectionInfo;

        osg::ref_ptr<osg::StateSet> _surfaceStateSet;
        osg::ref_ptr<osg::StateSet> _imageLayerStateSet;
    };

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_DRIVERS_REX_TERRAIN_ENGINE_ENGINE_NODE_H

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
#ifndef OSGEARTH_REX_LOAD_TILE_DATA
#define OSGEARTH_REX_LOAD_TILE_DATA 1

#include "Common"
#include "Loader"
#include "RenderBindings"
#include "TileNode"
#include "EngineContext"
#include "TileRenderModel"
#include <osgEarth/Progress>

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    class LoadTileData : public Loader::Request
    {
    public:
        LoadTileData(TileNode* tilenode, EngineContext* factory);

        CreateTileModelFilter& filter() { return _filter; }

        void setEnableCancelation(bool value) { _enableCancel = value; }

    public: // Loader::Request

        /** Fetches the data for the tile node. */
        void invoke(ProgressCallback*);

        /** Applies the fetched data to the tile node (scene-graph safe) */
        void apply(const osg::FrameStamp*);

        //! Creates a stateset containing GL compilable objects from the model
        osg::StateSet* createStateSet() const;

    protected:
        osg::observer_ptr<TileNode> _tilenode;
        osg::observer_ptr<TerrainEngineNode> _engine;
        osg::observer_ptr<EngineContext> _context;
        osg::ref_ptr<TerrainTileModel> _dataModel;
        TileRenderModel _renderModel;
        CreateTileModelFilter _filter;
        osg::observer_ptr< const Map > _map;
        bool _enableCancel;

        virtual ~LoadTileData() { }
    };

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_REX_LOAD_TILE_DATA

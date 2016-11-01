/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "LoadTileData"
#include "SurfaceNode"
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Terrain>
#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[LoadTileData] "


LoadTileData::LoadTileData(TileNode* tilenode, EngineContext* context) :
_tilenode(tilenode),
_context(context)
{
    //nop
}


// invoke runs in the background pager thread.
void
LoadTileData::invoke()
{
    osg::ref_ptr<TileNode> tilenode;
    if ( _tilenode.lock(tilenode) )
    {
        // Assemble all the components necessary to display this tile
        _dataModel = _context->getEngine()->createTileModel(
            _context->getMapFrame(),
            tilenode->getKey(),
            0L ); // progress
    }
}


// apply() runs in the update traversal and can safely alter the scene graph
void
LoadTileData::apply(const osg::FrameStamp* stamp)
{
    if ( _dataModel.valid() )
    {
        osg::ref_ptr<TileNode> tilenode;
        if ( _tilenode.lock(tilenode) )
        {
            const RenderBindings& bindings      = _context->getRenderBindings();
            const SelectionInfo&  selectionInfo = _context->getSelectionInfo();
            const MapInfo&        mapInfo       = _context->getMapFrame().getMapInfo();

            // Merge the new data into the tile.
            tilenode->merge(_dataModel.get(), bindings);

            // Mark as complete. TODO: per-data requests will do something different.
            tilenode->setDirty( false );

            // Notify listeners that we've added a tile.
            _context->getEngine()->getTerrain()->notifyTileAdded( _key, tilenode->getSurfaceNode() );

            OE_DEBUG << LC << "apply " << _dataModel->getKey().str() << "\n";

            // Delete the model immediately
            _dataModel = 0L;
        }
        else
        {
            OE_DEBUG << LC << "LoadTileData failed; TileNode disappeared\n";
        }
    }
}

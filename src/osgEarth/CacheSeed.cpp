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

#include <osgEarth/CacheSeed>
#include <osgEarth/CacheEstimator>
#include <osgEarth/MapFrame>
#include <OpenThreads/ScopedLock>
#include <limits.h>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;

CacheTileHandler::CacheTileHandler( TerrainLayer* layer, Map* map ):
_layer( layer ),
_map( map )
{
}

bool CacheTileHandler::handleTile(const TileKey& key, const TileVisitor& tv)
{        
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    // Just call createImage or createHeightField on the layer and the it will be cached!
    if (imageLayer)
    {                
        GeoImage image = imageLayer->createImage( key );
        if (image.valid())
        {                
            return true;
        }            
    }
    else if (elevationLayer )
    {
        GeoHeightField hf = elevationLayer->createHeightField( key );
        if (hf.valid())
        {                
            return true;
        }            
    }
    return false;        
}   

bool CacheTileHandler::hasData( const TileKey& key ) const
{
    TileSource* ts = _layer->getTileSource();
    if (ts)
    {
        return ts->hasData(key);
    }
    return true;
}

std::string CacheTileHandler::getProcessString() const
{
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    std::stringstream buf;
    buf << "osgearth_cache --seed ";
    if (imageLayer)
    {        
        for (int i = 0; i < _map->getNumImageLayers(); i++)
        {
            if (imageLayer == _map->getImageLayerAt(i))
            {
                buf << " --image " << i << " ";
                break;
            }
        }
    }
    else if (elevationLayer)
    {
        for (int i = 0; i < _map->getNumElevationLayers(); i++)
        {
            if (elevationLayer == _map->getElevationLayerAt(i))
            {
                buf << " --elevation " << i << " ";
                break;
            }
        }
    }
    return buf.str();
}



/***************************************************************************************/

CacheSeed::CacheSeed():
_visitor(new TileVisitor())
{
}

TileVisitor* CacheSeed::getVisitor() const
{
    return _visitor;
}

void CacheSeed::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}

void CacheSeed::run( TerrainLayer* layer, Map* map )
{
    _visitor->setTileHandler( new CacheTileHandler( layer, map ) );
    _visitor->run( map->getProfile() );
}
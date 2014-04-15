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
#include <osgEarth/TileHandler>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>

using namespace osgEarth;

bool TileHandler::handleTile(const TileKey& key)
{
    return true;
}


/*****************************************************************************************/
CacheTileHandler::CacheTileHandler( TerrainLayer* layer ):
_layer( layer )
{
}

bool CacheTileHandler::handleTile( const TileKey& key )
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
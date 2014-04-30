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
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>

#include <osgDB/WriteFile>
#include <osgDB/FileUtils>


using namespace osgEarth;

bool TileHandler::handleTile(const TileKey& key)
{
    return true;
}

bool TileHandler::hasData( const TileKey& key ) const
{
    return true;
}
        
std::string TileHandler::getProcessString() const
{
    return "";
}


/*****************************************************************************************/
CacheTileHandler::CacheTileHandler( TerrainLayer* layer, Map* map ):
_layer( layer ),
_map( map )
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
    buf << "osgearth_cache2 --seed ";
    if (imageLayer)
    {        
        for (unsigned int i = 0; i < _map->getNumImageLayers(); i++)
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
        for (unsigned int i = 0; i < _map->getNumElevationLayers(); i++)
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


/*****************************************************************************************/
// A global mutex to make sure that we're not creating directories concurrently.
// If you don't do this you will get errors when writing from multiple threads
WriteTMSTileHandler::WriteTMSTileHandler(TerrainLayer* layer, const std::string& destination, const std::string& extension):
_layer( layer ),
    _destination( destination ),
    _extension(extension),
    _width(0),
    _height(0),
    _maxLevel(0),
    _elevationPixelDepth(32)
{
}

const std::string& WriteTMSTileHandler::getExtension() const
{
    return _extension; 
}

const std::string& WriteTMSTileHandler::getDestination() const
{
    return _destination;
}

unsigned int WriteTMSTileHandler::getWidth() const
{
    return _width;
}

unsigned int WriteTMSTileHandler::getHeight() const
{
    return _height;
}

 unsigned int WriteTMSTileHandler::getMaxLevel() const
 {
     return _maxLevel;
 }

TerrainLayer* WriteTMSTileHandler::getLayer()
{
    return _layer; 
}

void WriteTMSTileHandler::setElevationPixelDepth(unsigned value)
{
    _elevationPixelDepth = value;
}

unsigned WriteTMSTileHandler::getElevationPixelDepth() const
{
    return _elevationPixelDepth;
}

osgDB::Options* WriteTMSTileHandler::getOptions() const
{
    return _options;
}

void WriteTMSTileHandler::setOptions(osgDB::Options* options)
{
    _options = options;
}

std::string WriteTMSTileHandler::getPathForTile( const TileKey &key )
{
    std::string layerFolder = toLegalFileName( _layer->getName() );         
    unsigned w, h;
    key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );         

    return Stringify() 
        << _destination 
        << "/" << layerFolder
        << "/" << key.getLevelOfDetail() 
        << "/" << key.getTileX() 
        << "/" << h - key.getTileY() - 1
        << "." << _extension;
}


bool WriteTMSTileHandler::handleTile( const TileKey& key )
{    
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );

    if (imageLayer)
    {                
        GeoImage geoImage = imageLayer->createImage( key );

        if (geoImage.valid())
        {                
            if (_width == 0 || _height == 0)
            {
                _width = geoImage.getImage()->s();
                _height = geoImage.getImage()->t();
            }
            // OE_NOTICE << "Created image for " << key.str() << std::endl;
            osg::ref_ptr< const osg::Image > final = geoImage.getImage();            

            // Get the path to write to
            std::string path = getPathForTile( key );

            // attempt to create the output folder:        
            osgEarth::makeDirectoryForFile( path );       

            // convert to RGB if necessary            
            if ( _extension == "jpg" && final->getPixelFormat() != GL_RGB )
            {
                final = ImageUtils::convertToRGB8( final );
            }
            if (key.getLevelOfDetail() > _maxLevel)
            {
                _maxLevel = key.getLevelOfDetail();
            }
            return osgDB::writeImageFile(*final, path, _options.get());
        }            
    }
    else if (elevationLayer )
    {
        GeoHeightField hf = elevationLayer->createHeightField( key );
        if (hf.valid())
        {
            if (_width == 0 || _height == 0)
            {
                _width = hf.getHeightField()->getNumColumns();
                _height = hf.getHeightField()->getNumRows();
            }
            // convert the HF to an image
            ImageToHeightFieldConverter conv;
            osg::ref_ptr< osg::Image > image = conv.convert( hf.getHeightField(), _elevationPixelDepth );				
            if (key.getLevelOfDetail() > _maxLevel)
            {
                _maxLevel = key.getLevelOfDetail();
            }

            // Get the path to write to
            std::string path = getPathForTile( key );

            // attempt to create the output folder:        
            osgEarth::makeDirectoryForFile( path );       


            return osgDB::writeImageFile(*image.get(), path, _options.get());
        }            
    }
    return false;        
} 

bool WriteTMSTileHandler::hasData( const TileKey& key ) const
{
    TileSource* ts = _layer->getTileSource();
    if (ts)
    {
        return ts->hasData(key);
    }
    return true;
}
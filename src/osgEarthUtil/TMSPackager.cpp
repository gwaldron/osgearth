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
#include <osgEarthUtil/TMSPackager>
#include <osgEarthUtil/TMS>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/TaskService>
#include <osgEarth/FileUtils>
#include <osgEarth/CacheEstimator>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>


#define LC "[TMSPackager] "

using namespace osgEarth::Util;
using namespace osgEarth;

WriteTMSTileHandler::WriteTMSTileHandler(TerrainLayer* layer,  Map* map, TMSPackager* packager):
    _layer( layer ),
    _map(map),
    _packager(packager)
{
}

std::string WriteTMSTileHandler::getPathForTile( const TileKey &key )
{
    std::string layerFolder = toLegalFileName( _packager->getLayerName() );         
    unsigned w, h;
    key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );         

    return Stringify() 
        << _packager->getDestination()
        << "/" << layerFolder
        << "/" << key.getLevelOfDetail() 
        << "/" << key.getTileX() 
        << "/" << h - key.getTileY() - 1
        << "." << _packager->getExtension();
}


bool WriteTMSTileHandler::handleTile(const TileKey& key, const TileVisitor& tv)
{    
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );

    // Get the path to write to
    std::string path = getPathForTile( key );

    // Don't write out a new file if we're not overwriting
    if (osgDB::fileExists(path) && !_packager->getOverwrite())
        return true;

    // attempt to create the output folder:        
    osgEarth::makeDirectoryForFile( path );       


    if (imageLayer)
    {                        
        GeoImage geoImage = imageLayer->createImage( key );

        if (geoImage.valid())
        {                             
            if (!_packager->getKeepEmpties() && ImageUtils::isEmptyImage(geoImage.getImage()))
            {
                OE_INFO << "Not writing completely transparent image for key " << key.str() << std::endl;
                return false;
            }

            // mask out areas not included in the request:
            for(std::vector<GeoExtent>::const_iterator g = tv.getExtents().begin();
                g != tv.getExtents().end();
                ++g)
            {
                geoImage.applyAlphaMask( *g );
            }

            // OE_NOTICE << "Created image for " << key.str() << std::endl;
            osg::ref_ptr< const osg::Image > final = geoImage.getImage();                        

            // convert to RGB if necessary            
            if ( _packager->getExtension() == "jpg" && final->getPixelFormat() != GL_RGB )
            {
                final = ImageUtils::convertToRGB8( final );
            }            
            return osgDB::writeImageFile(*final, path, _packager->getOptions());
        }            
    }
    else if (elevationLayer )
    {
        GeoHeightField hf = elevationLayer->createHeightField( key );
        if (hf.valid())
        {
            // convert the HF to an image
            ImageToHeightFieldConverter conv;
            osg::ref_ptr< osg::Image > image = conv.convert( hf.getHeightField(), _packager->getElevationPixelDepth() );				            
            return osgDB::writeImageFile(*image.get(), path, _packager->getOptions());
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

std::string WriteTMSTileHandler::getProcessString() const
{
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    std::stringstream buf;
    buf << "osgearth_package --tms ";
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

    // Options
    buf << " --out " << _packager->getDestination() << " ";
    buf << " --ext " << _packager->getExtension() << " ";
    buf << " --elevation-pixel-depth " << _packager->getElevationPixelDepth() << " ";
    if (_packager->getOptions())
    {
        buf << " --db-options " << _packager->getOptions()->getOptionString() << " ";    
    }
    if (_packager->getOverwrite())
    {
        buf << " --overwrite ";
    }            
    return buf.str();
}


/*****************************************************************************************************/

TMSPackager::TMSPackager():
_visitor(new TileVisitor()),
    _extension(""),
    _destination("out"),
    _elevationPixelDepth(32),
    _width(0),
    _height(0),
    _overwrite(false),
    _keepEmpties(false)
{
}

const std::string& TMSPackager::getDestination() const
{
    return _destination;
}

void TMSPackager::setDestination( const std::string& destination)
{
    _destination = destination;
}

const std::string& TMSPackager::getExtension() const
{
    return _extension;
}

void TMSPackager::setExtension( const std::string& extension)
{
    _extension = extension;
}

 void TMSPackager::setElevationPixelDepth(unsigned value)
 {
     _elevationPixelDepth = value;
 }

 unsigned TMSPackager::getElevationPixelDepth() const
 {
     return _elevationPixelDepth;
 }

osgDB::Options* TMSPackager::getOptions() const
{
    return _writeOptions.get();
}

void TMSPackager::setWriteOptions( osgDB::Options* options )
{
    _writeOptions = options;
}

const std::string& TMSPackager::getLayerName() const
{
    return _layerName;
}

void TMSPackager::setLayerName( const std::string& name)
{
    _layerName = name;
}

bool TMSPackager::getOverwrite() const
{
    return _overwrite;
}

void TMSPackager::setOverwrite(bool overwrite)
{
    _overwrite = overwrite;
}

bool TMSPackager::getKeepEmpties() const
{
    return _keepEmpties;
}

void TMSPackager::setKeepEmpties(bool keepEmpties)
{
    _keepEmpties = keepEmpties;
}

TileVisitor* TMSPackager::getTileVisitor() const
{
    return _visitor;
}

void TMSPackager::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}    

void TMSPackager::run( TerrainLayer* layer,  Map* map  )
{    
    // Get a test image from the root keys

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    map->getProfile()->getRootKeys( rootKeys );

    // fetch one tile to see what the image size should be
    ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);

    // Come up with a default name for the layer if it doesn't already have one.
    if (layer->getName().empty())
    {
        std::stringstream layerName;

        unsigned int index = 0;
        if (imageLayer)
        {            
            layerName << "image";
            // Get the index of the layer
            for (unsigned int i = 0; i < map->getNumImageLayers(); i++)
            {
                if (map->getImageLayerAt(i) == imageLayer)
                {
                    index = i;
                    break;
                }
            }            
        }
        else if (elevationLayer)
        {
            layerName << "elevation";
            // Get the index of the layer
            for (unsigned int i = 0; i < map->getNumElevationLayers(); i++)
            {
                if (map->getElevationLayerAt(i) == elevationLayer)
                {
                    index = i;
                    break;
                }
            }
        }
        layerName << index+1;
        OE_NOTICE << "Setting layer name to " << layerName.str() << std::endl;
        setLayerName(layerName.str());
    }
    else
    {
        setLayerName(layer->getName());
    }



    if (imageLayer)
    {
        GeoImage testImage;
        for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testImage.valid(); ++i )
        {
            testImage = imageLayer->createImage( *i );
        }
        if (testImage.valid())
        {
            _width = testImage.getImage()->s();
            _height = testImage.getImage()->t();

            bool alphaChannelRequired =
                ImageUtils::hasAlphaChannel(testImage.getImage()) ||
                _visitor->getExtents().size() > 0;

            // Figure out the extension if we haven't already assigned one.
            if (_extension.empty())
            {
                if (alphaChannelRequired)
                {
                    _extension = "png";
                }
                else
                {
                    _extension = "jpg";
                }
            }
            else if (_extension == "jpg" && alphaChannelRequired)
            {
                _extension = "png";
                OE_NOTICE << LC << "Extension changed to PNG since output requires an alpha channel" << std::endl;
            }

            OE_INFO << LC << "Output extension: " << _extension << std::endl;
        }
    }
    else if (elevationLayer)
    {
        // We must use tif no matter what with elevation layers.  It's the only format that currently can read/write single band imagery.
        _extension = "tif";
        GeoHeightField testHF;
        for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testHF.valid(); ++i )
        {
            testHF = elevationLayer->createHeightField( *i );
        }

        if (testHF.valid())
        {
            _width = testHF.getHeightField()->getNumColumns();
            _height = testHF.getHeightField()->getNumRows();
        }
    }


    _handler = new WriteTMSTileHandler(layer, map, this);    
    _visitor->setTileHandler( _handler );    
    _visitor->run( map->getProfile() );    
}

void TMSPackager::writeXML( TerrainLayer* layer, Map* map)
{
     // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        map->getProfile(),        
        _extension,
        _width,
        _height
        );

    std::string mimeType;
    if ( _extension == "png" )
        mimeType = "image/png";
    else if ( _extension == "jpg" || _extension == "jpeg" )
        mimeType = "image/jpeg";
    else if ( _extension == "tif" || _extension == "tiff" )
        mimeType = "image/tiff";
    else {
        OE_WARN << LC << "Unable to determine mime-type for extension \"" << _extension << "\"" << std::endl;
    }


    //TODO:  Fix
    unsigned int maxLevel = 23;
    tileMap->setTitle( _layerName );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::min(23u, maxLevel+1) );
    

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths( osgDB::concatPaths(_destination, toLegalFileName( _layerName )), "tms.xml");
    OE_NOTICE << "Layer name " << _layerName << std::endl;
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );
}
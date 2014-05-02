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
#include <osgEarthUtil/TMSPackager>
#include <osgEarthUtil/TMS>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/TaskService>
#include <osgEarth/CacheEstimator>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>


#define LC "[TMSPackager] "

using namespace osgEarth::Util;
using namespace osgEarth;

TMSPackager::TMSPackager():
_visitor(new TileVisitor()),
    _extension(""),
    _destination("out"),
    _elevationPixelDepth(32),
    _width(0),
    _height(0)
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
            // Figure out the extension if we haven't already assigned one.
            if (_extension.empty())
            {
                if (!ImageUtils::hasAlphaChannel(testImage.getImage()))
                {
                    _extension = "jpg";
                }
                else
                {
                    _extension = "png";
                }
                OE_INFO << "Selected extension" << _extension << std::endl;
            }

        }
    }
    else if (elevationLayer)
    {
        GeoHeightField testHF;
        for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testHF.valid(); ++i )
        {
            testHF = elevationLayer->createHeightField( *i );
        }

        if (testHF.valid())
        {
            _width = testHF.getHeightField()->getNumColumns();
            _width = testHF.getHeightField()->getNumRows();
        }
    }


    _handler = new WriteTMSTileHandler(layer, map, _destination, _extension);
    _handler->setElevationPixelDepth( _elevationPixelDepth );
    _handler->setOptions(_writeOptions.get());
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
    unsigned int maxLevel = 23;//_handler->getMaxLevel();
    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::min(23u, maxLevel+1) );
    

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths( osgDB::concatPaths(_destination, toLegalFileName( layer->getName() )), "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );
}
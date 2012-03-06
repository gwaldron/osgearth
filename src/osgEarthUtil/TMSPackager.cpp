/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>

#define LC "[TMSPackager] "

using namespace osgEarth::Util;
using namespace osgEarth;


TMSPackager::TMSPackager(const Profile* outProfile) :
_outProfile  ( outProfile ),
_maxLevel    ( 5 ),
_verbose     ( false ),
_abortOnError( true )
{
    //nop
}


TMSPackager::Result
TMSPackager::packageImageTile(ImageLayer*          layer,
                              const TileKey&       key,
                              const std::string&   rootDir,
                              const std::string&   extension,
                              unsigned&            out_maxLevel )
{
    unsigned w, h;
    key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

    GeoImage image = layer->createImage( key );
    if ( image.valid() )
    {
        std::string path = Stringify() 
            << rootDir 
            << "/" << key.getLevelOfDetail() 
            << "/" << key.getTileX() 
            << "/" << h - key.getTileY() - 1
            << "." << extension;

        // convert to RGB if necessary
        osg::ref_ptr<osg::Image> final = image.getImage();
        if ( extension == "jpg" && final->getPixelFormat() != GL_RGB )
            final = ImageUtils::convertToRGB8( image.getImage() );

        // dump it to disk
        osgDB::makeDirectoryForFile( path );
        bool writeOK = osgDB::writeImageFile( *final.get(), path );

        // increment the maximum detected tile level:
        if ( writeOK && key.getLevelOfDetail() > out_maxLevel )
            out_maxLevel = key.getLevelOfDetail();

        if ( _verbose )
        {
            if ( writeOK ) {
                OE_NOTICE << LC << "Wrote tile " << key.str() << std::endl;
            }
            else {
                OE_NOTICE << LC << "Error write tile " << key.str() << std::endl;
            }
        }

        if ( _abortOnError && !writeOK )
        {
            return Result( Stringify() << "Aborting, write failed for tile " << key.str() );
        }

        if ((key.getLevelOfDetail() + 1 < _maxLevel) &&
            (!layer->getImageLayerOptions().maxLevel().isSet() ||
             key.getLevelOfDetail() + 1 < *layer->getImageLayerOptions().maxLevel()))
        {
            for( unsigned q=0; q<4; ++q )
            {
                TileKey childKey = key.createChildKey(q);
                Result r = packageImageTile( layer, childKey, rootDir, extension, out_maxLevel );
                if ( _abortOnError && !r.ok )
                    return r;
            }
        }
    }

    return Result();
}


TMSPackager::Result
TMSPackager::packageElevationTile(ElevationLayer*      layer,
                                  const TileKey&       key,
                                  const std::string&   rootDir,
                                  const std::string&   extension,
                                  unsigned&            out_maxLevel)
{
    unsigned w, h;
    key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

    GeoHeightField hf = layer->createHeightField( key );
    if ( hf.valid() )
    {
        std::string path = Stringify() 
            << rootDir 
            << "/" << key.getLevelOfDetail() 
            << "/" << key.getTileX() 
            << "/" << h - key.getTileY() - 1
            << "." << extension;

        // convert the HF to an image
        ImageToHeightFieldConverter conv;
        osg::ref_ptr<osg::Image> image = conv.convert( hf.getHeightField() );

        // dump it to disk
        osgDB::makeDirectoryForFile( path );
        bool writeOK = osgDB::writeImageFile( *image.get(), path );

        // increment the maximum detected tile level:
        if ( writeOK && key.getLevelOfDetail() > out_maxLevel )
            out_maxLevel = key.getLevelOfDetail();

        if ( _verbose )
        {
            if ( writeOK ) {
                OE_NOTICE << LC << "Wrote tile " << key.str() << std::endl;
            }
            else {
                OE_NOTICE << LC << "Error write tile " << key.str() << std::endl;
            }
        }

        if ( _abortOnError && !writeOK )
        {
            return Result( Stringify() << "Aborting, write failed for tile " << key.str() );
        }

        if ((key.getLevelOfDetail() + 1 < _maxLevel) &&
            (!layer->getElevationLayerOptions().maxLevel().isSet() ||
             key.getLevelOfDetail() + 1 < *layer->getElevationLayerOptions().maxLevel()))
        {
            for( unsigned q=0; q<4; ++q )
            {
                TileKey childKey = key.createChildKey(q);
                Result r = packageElevationTile( layer, childKey, rootDir, extension, out_maxLevel );
                if ( _abortOnError && !r.ok )
                    return r;
            }
        }
    }

    return Result();
}


TMSPackager::Result
TMSPackager::package(ImageLayer*        layer,
                     const std::string& rootFolder,
                     const std::string& imageExtension)
{
    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    osgDB::makeDirectory( rootFolder );
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    _outProfile->getRootKeys( rootKeys );

    if ( rootKeys.size() == 0 )
        return Result( "Unable to calculate root key set" );

    // compute a mime type
    std::string mimeType;
    if ( imageExtension == "png" )
        mimeType = "image/png";
    else if ( imageExtension == "jpg" )
        mimeType = "image/jpeg";
    else if ( imageExtension == "tif" )
        mimeType = "image/tiff";
    else
        return Result( Stringify() << "Unable to determine mime-type for extension " << imageExtension );

    // fetch one tile to see what the image size should be
    
    GeoImage testImage;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testImage.valid(); ++i )
    {
        testImage = layer->createImage( *i );
    }
    if ( !testImage.valid() )
        return Result( "Unable to determine appropriate image type" );

    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        Result r = packageImageTile( layer, *i, rootFolder, imageExtension, maxLevel );
        if ( _abortOnError && !r.ok )
            return r;
    }

    // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        _outProfile.get(),
        imageExtension,
        testImage.getImage()->s(),
        testImage.getImage()->t() );

    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( maxLevel + 1 );

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths(rootFolder, "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );

    return Result();
}


TMSPackager::Result
TMSPackager::package(ElevationLayer*    layer,
                     const std::string& rootFolder)
{
    if ( !layer || !_outProfile.valid() )
        return Result( "Illegal null layer or profile" );

    // attempt to create the output folder:
    osgDB::makeDirectory( rootFolder );
    if ( !osgDB::fileExists( rootFolder ) )
        return Result( "Unable to create output folder" );

    // collect the root tile keys in preparation for packaging:
    std::vector<TileKey> rootKeys;
    _outProfile->getRootKeys( rootKeys );

    if ( rootKeys.size() == 0 )
        return Result( "Unable to calculate root key set" );

    std::string extension = "tif", mimeType = "image/tiff";

    // fetch one tile to see what the tile size will be
    GeoHeightField testHF = layer->createHeightField( rootKeys[0] );
    if ( !testHF.valid() )
        return Result( "Unable to determine heightfield size" );

    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        Result r = packageElevationTile( layer, *i, rootFolder, extension, maxLevel );
        if ( _abortOnError && !r.ok )
            return r;
    }

    // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        _outProfile.get(),
        extension,
        testHF.getHeightField()->getNumColumns(),
        testHF.getHeightField()->getNumRows() );

    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( maxLevel + 1 );

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths(rootFolder, "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );

    return Result();
}

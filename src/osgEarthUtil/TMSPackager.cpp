/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
_outProfile         ( outProfile ),
_maxLevel           ( 5 ),
_verbose            ( false ),
_overwrite          ( false ),
_keepEmptyImageTiles( false ),
_abortOnError       ( true )
{
    //nop
}


void
TMSPackager::addExtent( const GeoExtent& extent )
{
    _extents.push_back(extent);
}


bool
TMSPackager::shouldPackageKey( const TileKey& key ) const
{
    // if there are no extent filters, or we're at a sufficiently low level, 
    // always package the key.
    if ( _extents.size() == 0 || key.getLevelOfDetail() <= 1 )
        return true;

    // check for intersection with one of the filter extents.
    for( std::vector<GeoExtent>::const_iterator i = _extents.begin(); i != _extents.end(); ++i )
    {
        if ( i->intersects( key.getExtent() ) )
            return true;
    }

    return false;
}


TMSPackager::Result
TMSPackager::packageImageTile(ImageLayer*          layer,
                              const TileKey&       key,
                              const std::string&   rootDir,
                              const std::string&   extension,
                              unsigned&            out_maxLevel )
{
    unsigned minLevel = layer->getImageLayerOptions().minLevel().isSet() ?
        *layer->getImageLayerOptions().minLevel() : 0;
    

    if ( shouldPackageKey(key) && key.getLevelOfDetail() >= minLevel )
    {
        unsigned w, h;
        key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

        std::string path = Stringify() 
            << rootDir 
            << "/" << key.getLevelOfDetail() 
            << "/" << key.getTileX() 
            << "/" << h - key.getTileY() - 1
            << "." << extension;

        bool tileOK = osgDB::fileExists(path) && !_overwrite;
        if ( !tileOK )
        {
            GeoImage image = layer->createImage( key );
            if ( image.valid() )
            {
                // check for empty:
                if ( !_keepEmptyImageTiles && ImageUtils::isEmptyImage(image.getImage()) )
                {
                    if ( _verbose )
                    {
                        OE_NOTICE << LC << "Skipping empty tile " << key.str() << std::endl;
                    }
                }
                else
                {
                    // convert to RGB if necessary
                    osg::ref_ptr<osg::Image> final = image.getImage();
                    if ( extension == "jpg" && final->getPixelFormat() != GL_RGB )
                        final = ImageUtils::convertToRGB8( image.getImage() );

                    // dump it to disk
                    osgDB::makeDirectoryForFile( path );
                    tileOK = osgDB::writeImageFile( *final.get(), path );

                    if ( _verbose )
                    {
                        if ( tileOK ) {
                            OE_NOTICE << LC << "Wrote tile " << key.str() << " (" << key.getExtent().toString() << ")" << std::endl;
                        }
                        else {
                            OE_NOTICE << LC << "Error write tile " << key.str() << std::endl;
                        }
                    }

                    if ( _abortOnError && !tileOK )
                    {
                        return Result( Stringify() << "Aborting, write failed for tile " << key.str() );
                    }
                }
            }
        }
        else
        {
            if ( _verbose )
            {
                OE_NOTICE << LC << "Tile " << key.str() << " already exists" << std::endl;
            }
        }

        // increment the maximum detected tile level:
        if ( tileOK && key.getLevelOfDetail() > out_maxLevel )
        {
            out_maxLevel = key.getLevelOfDetail();
        }

        // see if subdivision should continue.
        unsigned lod = key.getLevelOfDetail();
        const ImageLayerOptions& options = layer->getImageLayerOptions();

        unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
        unsigned maxLevel = std::min(_maxLevel, layerMaxLevel);
        bool subdivide =
            (options.minLevel().isSet() && lod < *options.minLevel()) ||
            (tileOK && lod+1 < maxLevel);

        // subdivide if necessary:
        if ( subdivide )
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
    unsigned minLevel = layer->getElevationLayerOptions().minLevel().isSet() ?
        *layer->getElevationLayerOptions().minLevel() : 0;

    if ( shouldPackageKey(key) && key.getLevelOfDetail() >= minLevel )
    {
        unsigned w, h;
        key.getProfile()->getNumTiles( key.getLevelOfDetail(), w, h );

        std::string path = Stringify() 
            << rootDir 
            << "/" << key.getLevelOfDetail() 
            << "/" << key.getTileX() 
            << "/" << h - key.getTileY() - 1
            << "." << extension;

        bool tileOK = osgDB::fileExists(path) && !_overwrite;
        if ( !tileOK )
        {

            GeoHeightField hf = layer->createHeightField( key );
            if ( hf.valid() )
            {
                // convert the HF to an image
                ImageToHeightFieldConverter conv;
                osg::ref_ptr<osg::Image> image = conv.convert( hf.getHeightField() );

                // dump it to disk
                osgDB::makeDirectoryForFile( path );
                tileOK = osgDB::writeImageFile( *image.get(), path );

                if ( _verbose )
                {
                    if ( tileOK ) {
                        OE_NOTICE << LC << "Wrote tile " << key.str() << " (" << key.getExtent().toString() << ")" << std::endl;
                    }
                    else {
                        OE_NOTICE << LC << "Error write tile " << key.str() << std::endl;
                    }
                }

                if ( _abortOnError && !tileOK )
                {
                    return Result( Stringify() << "Aborting, write failed for tile " << key.str() );
                }
            }
        }
        else
        {
            if ( _verbose )
            {
                OE_NOTICE << LC << "Tile " << key.str() << " already exists" << std::endl;
            }
        }

        // increment the maximum detected tile level:
        if ( tileOK && key.getLevelOfDetail() > out_maxLevel )
        {
            out_maxLevel = key.getLevelOfDetail();
        }

        // see if subdivision should continue.
        unsigned lod = key.getLevelOfDetail();
        const ElevationLayerOptions& options = layer->getElevationLayerOptions();

        unsigned layerMaxLevel = (options.maxLevel().isSet()? *options.maxLevel() : 99);
        unsigned maxLevel = std::min(_maxLevel, layerMaxLevel);
        bool subdivide =
            (options.minLevel().isSet() && lod < *options.minLevel()) ||
            (tileOK && lod+1 < maxLevel);

        // subdivide if necessary:
        if ( subdivide )
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
                     const std::string& overrideExtension )
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

    // fetch one tile to see what the image size should be
    
    GeoImage testImage;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testImage.valid(); ++i )
    {
        testImage = layer->createImage( *i );
    }
    if ( !testImage.valid() )
        return Result( "Unable to get a test image!" );

    // try to determine the image extension:
    std::string extension = overrideExtension;

    if ( extension.empty() && testImage.valid() )
    {
        extension = toLower( osgDB::getFileExtension( testImage.getImage()->getFileName() ) );
        if ( extension.empty() )
        {
            if ( ImageUtils::hasAlphaChannel(testImage.getImage()) )
            {
                extension = "png";
            }
            else
            {
                extension = "jpg";
            }
        }
    }

    // compute a mime type
    std::string mimeType;
    if ( extension == "png" )
        mimeType = "image/png";
    else if ( extension == "jpg" || extension == "jpeg" )
        mimeType = "image/jpeg";
    else if ( extension == "tif" || extension == "tiff" )
        mimeType = "image/tiff";
    else {
        OE_WARN << LC << "Unable to determine mime-type for extension \"" << extension << "\"" << std::endl;
    }

    if ( _verbose )
    {
        OE_NOTICE << LC << "MIME-TYPE = " << mimeType << ", Extension = " << extension << std::endl;
    }

    // package the tile hierarchy
    unsigned maxLevel = 0;
    for( std::vector<TileKey>::const_iterator i = rootKeys.begin(); i != rootKeys.end(); ++i )
    {
        Result r = packageImageTile( layer, *i, rootFolder, extension, maxLevel );
        if ( _abortOnError && !r.ok )
            return r;
    }

    // create the tile map metadata:
    osg::ref_ptr<TMS::TileMap> tileMap = TMS::TileMap::create(
        "",
        _outProfile.get(),
        extension,
        testImage.getImage()->s(),
        testImage.getImage()->t() );

    tileMap->setTitle( layer->getName() );
    tileMap->setVersion( "1.0.0" );
    tileMap->getFormat().setMimeType( mimeType );
    tileMap->generateTileSets( std::max(23u, maxLevel+1) );

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
    if ( _verbose )
    {
        OE_NOTICE << LC << "MIME-TYPE = " << mimeType << ", Extension = " << extension << std::endl;
    }

    // fetch one tile to see what the tile size will be
    GeoHeightField testHF;
    for( std::vector<TileKey>::iterator i = rootKeys.begin(); i != rootKeys.end() && !testHF.valid(); ++i )
    {
        testHF = layer->createHeightField( *i );
    }
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
    tileMap->generateTileSets( std::max(23u, maxLevel+1) );

    // write out the tilemap catalog:
    std::string tileMapFilename = osgDB::concatPaths(rootFolder, "tms.xml");
    TMS::TileMapReaderWriter::write( tileMap.get(), tileMapFilename );

    return Result();
}

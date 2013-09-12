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
#include <osgEarthUtil/TMSBackFiller>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageMosaic>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#define LC "[TMSBackFiller] "

using namespace osgEarth::Util;
using namespace osgEarth;

TMSBackFiller::TMSBackFiller()
{
}


void TMSBackFiller::process( const std::string& tms, osgDB::Options* options )
{               
    std::string fullPath = getFullPath( "", tms );        
    _options = options;

    //Read the tilemap
    _tileMap = TileMapReaderWriter::read( fullPath, 0 );
    if (_tileMap)
    {                        
        //The max level is where we are going to read data from, so we need to start one level up.
        osg::ref_ptr< const Profile> profile = _tileMap->createProfile();           

        //If the bounds aren't valid just use the full extent of the profile.
        if (!_bounds.valid())
        {                
            _bounds = profile->getExtent().bounds();
        }


        int firstLevel = _maxLevel-1;            

        GeoExtent extent( profile->getSRS(), _bounds );           

        //Process each level in it's entirety
        for (int level = firstLevel; level >= static_cast<int>(_minLevel); level--)
        {
            if (_verbose) OE_NOTICE << "Processing level " << level << std::endl;                

            TileKey ll = profile->createTileKey(extent.xMin(), extent.yMin(), level);
            TileKey ur = profile->createTileKey(extent.xMax(), extent.yMax(), level);

            for (unsigned int x = ll.getTileX(); x <= ur.getTileX(); x++)
            {
                for (unsigned int y = ur.getTileY(); y <= ll.getTileY(); y++)
                {
                    TileKey key = TileKey(level, x, y, profile.get());
                    processKey( key );
                }
            }                

        }            
    }
    else
    {
        OE_NOTICE << "Failed to load TileMap from " << _tmsPath << std::endl;
    }
}

void TMSBackFiller::processKey( const TileKey& key )
{
    if (_verbose) OE_NOTICE << "Processing key " << key.str() << std::endl;

    //Get all of the child tiles for this key, load them and mosaic them into a new tile
    TileKey ulKey = key.createChildKey( 0 );
    TileKey urKey = key.createChildKey( 1 );
    TileKey llKey = key.createChildKey( 2 );
    TileKey lrKey = key.createChildKey( 3 );

    osg::ref_ptr< osg::Image > ul = readTile( ulKey );
    osg::ref_ptr< osg::Image > ur = readTile( urKey );
    osg::ref_ptr< osg::Image > ll = readTile( llKey );
    osg::ref_ptr< osg::Image > lr = readTile( lrKey );

    if (ul.valid() && ur.valid() && ll.valid() && lr.valid())
    {            
        //Merge them together
        ImageMosaic mosaic;
        mosaic.getImages().push_back( TileImage( ul.get(), ulKey ) );
        mosaic.getImages().push_back( TileImage( ur.get(), urKey ) );
        mosaic.getImages().push_back( TileImage( ll.get(), llKey ) );
        mosaic.getImages().push_back( TileImage( lr.get(), lrKey ) );            

        osg::ref_ptr< osg::Image> merged = mosaic.createImage();
        if (merged.valid())
        {
            //Resize the image so it's the same size as one of the input files
            osg::ref_ptr<osg::Image> resized;
            ImageUtils::resizeImage( merged.get(), ul->s(), ul->t(), resized );
            std::string outputFilename = getFilename( key );                
            writeTile( key, resized.get() );
        }
    }                
}    

std::string TMSBackFiller::getFilename( const TileKey& key )
{
    return _tileMap->getURL( key, false );        
}

osg::Image* TMSBackFiller::readTile( const TileKey& key )
{
    std::string filename = getFilename( key );        
    return osgDB::readImageFile( filename );        
}

void TMSBackFiller::writeTile( const TileKey& key, osg::Image* image )
{
    std::string filename = getFilename( key );
    if ( !osgDB::fileExists( osgDB::getFilePath(filename) ) )
        osgDB::makeDirectoryForFile( filename );
    osgDB::writeImageFile( *image, filename, _options.get() );        
}     
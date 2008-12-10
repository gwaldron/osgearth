/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/TileSource>
#include <osgDB/ReadFile>

using namespace osgEarth;

ReaderWriterTileSource::ReaderWriterTileSource(const std::string& _extension,
                                               const osgDB::ReaderWriter::Options* _options )
: extension( _extension ),
  options( _options )
{
}

osg::Image*
ReaderWriterTileSource::createImage( const TileKey* key )
{
    std::string uri = key->getName() + "." + extension;
    osg::Image* image = NULL;

    image = osgDB::readImageFile( uri, options.get() );
    if ( !image )
    {
        osg::notify(osg::WARN) << "ReaderWriterTileSource: osgDB::readImageFile FAILED for \"" << uri << "\"" << std::endl;
    }
    return image;
}

osg::HeightField*
ReaderWriterTileSource::createHeightField( const TileKey* key )
{
    std::string uri = key->getName() + "." + extension;
    osg::HeightField* field = NULL;

    field = osgDB::readHeightFieldFile( uri, options.get() );
    if ( !field )
    {
        osg::notify(osg::WARN) << "ReaderWriterTileSource: osgDB::readHeightField FAILED for \"" << uri << "\"" << std::endl;
    }
    return field;
}

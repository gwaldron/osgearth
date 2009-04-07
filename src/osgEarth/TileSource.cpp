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
#include <limits.h>

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarth/Mercator>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;


#define PROPERTY_MIN_LEVEL "min_level"
#define PROPERTY_MAX_LEVEL "max_level"


TileSource::TileSource():
_minLevel(0),
_maxLevel(25)
{
}

TileSource::~TileSource()
{
}

void
TileSource::init(const osgDB::ReaderWriter::Options* options)
{
    if ( options->getPluginData( PROPERTY_MIN_LEVEL ) )
        _minLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MIN_LEVEL ), 0 );

    if ( options->getPluginData( PROPERTY_MAX_LEVEL ) )
        _maxLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MAX_LEVEL ), INT_MAX );
}

osg::HeightField*
TileSource::createHeightField( const TileKey* key )
{
    osg::ref_ptr<osg::Image> image = createImage(key);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        hf = conv.convert( image.get() );
    }      
    return hf;
}

bool
TileSource::isKeyValid(const osgEarth::TileKey *key)
{
  if (!key) return false;

  //Check to see that the given tile is within the LOD range for this TileSource
  return ((key->getLevelOfDetail() >= _minLevel) && (key->getLevelOfDetail() <= _maxLevel));
}

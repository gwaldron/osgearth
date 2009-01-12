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

TileSource::TileSource():
_minLevel(0),
_maxLevel(INT_MAX)
{
}

TileSource::~TileSource()
{
}

const osgEarth::TileGridProfile&
TileSource::getProfile()
{
    return _profile;
}

#define PROPERTY_MIN_LEVEL "min_level"
#define PROPERTY_MAX_LEVEL "max_level"

void
TileSource::init(const osgDB::ReaderWriter::Options* options)
{
    if ( options->getPluginData( PROPERTY_MIN_LEVEL ) )
        _minLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MIN_LEVEL ), 0 );

    if ( options->getPluginData( PROPERTY_MAX_LEVEL ) )
        _maxLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MAX_LEVEL ), INT_MAX );
}
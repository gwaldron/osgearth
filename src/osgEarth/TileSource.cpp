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
#include <osgEarth/Registry>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;


#define PROPERTY_MIN_LEVEL    "min_level"
#define PROPERTY_MAX_LEVEL    "max_level"
#define PROPERTY_NODATA_VALUE "nodata_value"
#define PROPERTY_NODATA_MIN   "nodata_min"
#define PROPERTY_NODATA_MAX   "nodata_max"
#define PROPERTY_PROFILE      "profile"


#define DEFAULT_MIN_LEVEL 0
#define DEFAULT_MAX_LEVEL 25


TileSource::TileSource(const osgDB::ReaderWriter::Options* options) :
_options( options ),
_minLevel( DEFAULT_MIN_LEVEL ),
_maxLevel( DEFAULT_MAX_LEVEL ),
_noDataValue(SHRT_MIN),
_noDataMinValue(-FLT_MAX),
_noDataMaxValue(FLT_MAX),
_max_data_level(INT_MAX)
{
    if ( options )
    {
        if ( options->getPluginData( PROPERTY_MIN_LEVEL ) )
            _minLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MIN_LEVEL ), _minLevel );
   
        if ( options->getPluginData( PROPERTY_MAX_LEVEL ) )
            _maxLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MAX_LEVEL ), _maxLevel );

        if ( options->getPluginData( PROPERTY_NODATA_VALUE ) )
            _noDataValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_VALUE ), _noDataValue);

        if ( options->getPluginData( PROPERTY_NODATA_MIN ) )
            _noDataMinValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_MIN ), _noDataMinValue);

        if ( options->getPluginData( PROPERTY_NODATA_MAX ) )
            _noDataMaxValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_MAX ), _noDataMaxValue);
    }
}

TileSource::~TileSource()
{
    //NOP
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
TileSource::isOK() const 
{
    return getProfile() != NULL;
}

bool
TileSource::isKeyValid(const osgEarth::TileKey *key)
{
  if (!key) return false;

  //Check to see that the given tile is within the LOD range for this TileSource
  return ((key->getLevelOfDetail() >= _minLevel) && (key->getLevelOfDetail() <= _maxLevel));
}

void
TileSource::setOverrideProfile( const Profile* overrideProfile )
{
    _profile = overrideProfile;
}

const Profile*
TileSource::initProfile( const Profile* mapProfile, const std::string& configPath )
{
    // we always call createProfile, but only USE its return value if the user did
    // not supply an override profile. Why? because the user might do other driver-
    // initialization things in the createProfile code.
    osg::ref_ptr<const Profile> new_profile = createProfile( mapProfile, configPath );
    if ( !_profile.valid() )
    {
        _profile = new_profile.get();
    }

    return _profile.get();
}

const Profile*
TileSource::getProfile() const
{
    return _profile.get();
}

const osgDB::ReaderWriter::Options*
TileSource::getOptions() const
{
    return _options.get();
}

float
TileSource::getNoDataValue()
{
    return _noDataValue;
}

void
TileSource::setNoDataValue(float noDataValue)
{
    _noDataValue;
}

float
TileSource::getNoDataMinValue()
{
    return _noDataMinValue;
}

void
TileSource::setNoDataMinValue(float noDataMinValue)
{
    _noDataMinValue = noDataMinValue;
}

float
TileSource::getNoDataMaxValue()
{
    return _noDataMaxValue;
}

void 
TileSource::setNoDataMaxValue(float noDataMaxValue)
{
    _noDataMaxValue = noDataMaxValue;
}

const GeoExtent&
TileSource::getDataExtent() const
{
    return _data_extent.defined() ? _data_extent : _profile->getExtent();
}

void 
TileSource::setDataExtent( const GeoExtent& extent )
{
    _data_extent = extent;
}

unsigned int
TileSource::getMaxDataLevel() const
{
    return _max_data_level;
}

void
TileSource::setMaxDataLevel( unsigned int value )
{
    _max_data_level = value;
}

bool
TileSource::hasPersistentCache() const
{
    return false;
}

bool
TileSource::supportsPersistentCaching() const
{
    return true;
}

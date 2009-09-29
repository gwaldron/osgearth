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
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;


#define PROPERTY_NODATA_VALUE "nodata_value"
#define PROPERTY_NODATA_MIN   "nodata_min"
#define PROPERTY_NODATA_MAX   "nodata_max"
#define PROPERTY_PROFILE      "profile"

TileSource::TileSource(const osgDB::ReaderWriter::Options* options) :
_options( options ),
_noDataValue(SHRT_MIN),
_noDataMinValue(-FLT_MAX),
_noDataMaxValue(FLT_MAX),
_max_data_level(INT_MAX)
{
    if ( options )
    {
        if ( options->getPluginData( PROPERTY_NODATA_VALUE ) )
            _noDataValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_VALUE ), _noDataValue);

        if ( options->getPluginData( PROPERTY_NODATA_MIN ) )
            _noDataMinValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_MIN ), _noDataMinValue);

        if ( options->getPluginData( PROPERTY_NODATA_MAX ) )
            _noDataMaxValue = as<float>( (const char*)options->getPluginData( PROPERTY_NODATA_MAX ), _noDataMaxValue);
    }

	//Create a memory cache
	_memCache = new MemCache();
}

TileSource::~TileSource()
{
    //NOP
}


osg::Image*
TileSource::getImage( const TileKey* key)
{
	//Try to get it from the memcache fist
	osg::Image* image = NULL;
	if (_memCache.valid())
	{
		image = _memCache->getImage( key,"","");
	}

	if (!image)
	{
		image = createImage( key );
		if (image && _memCache.valid())
		{
			_memCache->setImage( key, "", "",image );
		}
	}
	return image;
}

osg::HeightField*
TileSource::getHeightField( const TileKey* key)
{
	osg::HeightField* hf = NULL;
	if (_memCache.valid())
	{
		hf = _memCache->getHeightField( key, "", "" );
	}

	if (!hf)
	{
		hf = createHeightField( key );
		if (hf && _memCache.valid())
		{
			_memCache->setHeightField( key, "", "", hf );
		}
	}
	return hf;
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

void
TileSource::setProfile( const Profile* profile )
{
    _profile = profile;
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
TileSource::supportsPersistentCaching() const
{
    return true;
}
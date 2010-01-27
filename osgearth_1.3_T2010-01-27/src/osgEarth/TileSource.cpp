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

#define PROPERTY_NAME         "name"
#define PROPERTY_NODATA_VALUE "nodata_value"
#define PROPERTY_NODATA_MIN   "nodata_min"
#define PROPERTY_NODATA_MAX   "nodata_max"
#define PROPERTY_PROFILE      "profile"

TileSource::TileSource(const PluginOptions* options) :
_options( options ),
_noDataValue(SHRT_MIN),
_noDataMinValue(-FLT_MAX),
_noDataMaxValue(FLT_MAX),
_max_data_level(INT_MAX)
{
    this->setThreadSafeRefUnref( true );

    if ( options )
    {
        const Config& conf = options->config();
        _name = conf.value( PROPERTY_NAME );
        _noDataValue    = conf.value<float>( PROPERTY_NODATA_VALUE, _noDataValue );
        _noDataMinValue = conf.value<float>( PROPERTY_NODATA_MIN, _noDataMinValue );
        _noDataMaxValue = conf.value<float>( PROPERTY_NODATA_MAX, _noDataMaxValue );
    }

	_memCache = new MemCache();
}

TileSource::~TileSource()
{
    //NOP
}


osg::Image*
TileSource::getImage( const TileKey* key,
                      ProgressCallback* progress
                      )
{
	//Try to get it from the memcache fist
    osg::ref_ptr<osg::Image> image = NULL;
	if (_memCache.valid())
	{
		image = _memCache->getImage( key,"","");
	}

	if (!image.valid())
	{
		image = createImage( key, progress );
		if (image.valid() && _memCache.valid())
		{
			_memCache->setImage( key, "", "",image.get() );
		}
	}
	return image.release();
}

osg::HeightField*
TileSource::getHeightField( const TileKey* key,
                            ProgressCallback* progress
                           )
{
    osg::ref_ptr<osg::HeightField> hf = NULL;
	if (_memCache.valid())
	{
		hf = _memCache->getHeightField( key, "", "" );
	}

	if (!hf.valid())
	{
		hf = createHeightField( key, progress );
		if (hf.valid() && _memCache.valid())
		{
			_memCache->setHeightField( key, "", "", hf.get() );
		}
	}
	return hf.release();
}

osg::HeightField*
TileSource::createHeightField( const TileKey* key,
                               ProgressCallback* progress)
{
    osg::ref_ptr<osg::Image> image = createImage(key, progress);
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

const PluginOptions*
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
    _noDataValue = noDataValue;
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
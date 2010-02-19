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


/****************************************************************/

TileSourceOptions::TileSourceOptions( const PluginOptions* po ) :
DriverOptions( po ),
_tileSize( 256 ),
_noDataValue( (float)SHRT_MIN ),
_noDataMinValue( -FLT_MAX ),
_noDataMaxValue( FLT_MAX )
{
    config().getIfSet( "tile_size", _tileSize );
    config().getIfSet( "nodata_value", _noDataValue );
    config().getIfSet( "nodata_min", _noDataMinValue );
    config().getIfSet( "nodata_max", _noDataMaxValue );
}

Config
TileSourceOptions::toConfig() const
{ 
    Config conf = DriverOptions::toConfig();
    conf.updateIfSet( "tile_size", _tileSize );
    conf.updateIfSet( "nodata_value", _noDataValue );
    conf.updateIfSet( "nodata_min", _noDataMinValue );
    conf.updateIfSet( "nodata_max", _noDataMaxValue );
    return conf;
}

/****************************************************************/  

TileSource::TileSource(const PluginOptions* options) :
_maxDataLevel( INT_MAX )
{
    this->setThreadSafeRefUnref( true );

    _settings = dynamic_cast<const TileSourceOptions*>( options );
    if ( !_settings.valid() )
        _settings = new TileSourceOptions( options );

    _memCache = new MemCache();
}

int
TileSource::getPixelsPerTile() const
{
    return _settings->tileSize().value();
}

osg::Image*
TileSource::getImage( const TileKey* key,
                      ProgressCallback* progress )
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

const GeoExtent&
TileSource::getDataExtent() const
{
    return _dataExtent.defined() ? _dataExtent : _profile->getExtent();
}

void 
TileSource::setDataExtent( const GeoExtent& extent )
{
    _dataExtent = extent;
}

unsigned int
TileSource::getMaxDataLevel() const
{
    return _maxDataLevel;
}

void
TileSource::setMaxDataLevel( unsigned int value )
{
    _maxDataLevel = value;
}

bool
TileSource::supportsPersistentCaching() const
{
    return true;
}



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

#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/Script>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarth/FileUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/StringUtils>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/TextureRectangle>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

//---------------------------------------------------------------------------

Session::Session( const Map* map, StyleSheet* styles, const osgDB::Options* dbOptions ) :
osg::Referenced( true ),
_map           ( map ),
_mapInfo       ( map ),
_dbOptions     ( dbOptions )
{
    if ( styles )
        setStyles( styles );
    else
        _styles = new StyleSheet();

    // if the caller did not provide a dbOptions, take it from the map.
    if ( map && !dbOptions )
        _dbOptions = map->getDBOptions();
}

Session::~Session()
{
}

const osgDB::Options*
Session::getDBOptions() const
{
    return _dbOptions.get();
}

MapFrame
Session::createMapFrame( Map::ModelParts parts ) const
{
    return MapFrame( _map.get(), parts );
}

void
Session::putObject( const std::string& key, osg::Referenced* object )
{
    Threading::ScopedWriteLock lock( _objMapMutex );
    _objMap[key] = object;
}

void
Session::removeObject( const std::string& key )
{
    Threading::ScopedWriteLock lock( _objMapMutex );
    _objMap.erase( key );
}

void
Session::setStyles( StyleSheet* value )
{
    _styles = value ? value : new StyleSheet();

    // Go ahead and create the script engine for the StyleSheet
    if (_styles && _styles->script())
      _styleScriptEngine = ScriptEngineFactory::create(Script(_styles->script()->code, _styles->script()->language, _styles->script()->name));
    else
      _styleScriptEngine = 0L;
}

ScriptEngine*
Session::getScriptEngine() const
{
  return _styleScriptEngine.get();
}
FeatureSource *
Session::getFeatureSource() const 
{ 
	return _featureSource.get(); 
}

void 
Session::setFeatureSource(FeatureSource * source)
{ 
	_featureSource = source;
}

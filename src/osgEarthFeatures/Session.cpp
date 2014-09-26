/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthSymbology/ResourceCache>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/TextureRectangle>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

//---------------------------------------------------------------------------

Session::Session( const Map* map, StyleSheet* styles, FeatureSource* source, const osgDB::Options* dbOptions ) :
osg::Referenced( true ),
_map           ( map ),
_mapInfo       ( map ),
_featureSource ( source ),
_dbOptions     ( dbOptions )
{
    if ( styles )
        setStyles( styles );
    else
        _styles = new StyleSheet();

    // if the caller did not provide a dbOptions, take it from the map.
    if ( map && !dbOptions )
        _dbOptions = map->getDBOptions();

    // A new cache to optimize state changes. Since the cache lives in the Session, any
    // geometry created under this session takes advantage of it. That's reasonable since
    // tiles in a particular "layer" will tend to share state.
    _stateSetCache = new StateSetCache();
}

Session::~Session()
{
    //nop
}

const osgDB::Options*
Session::getDBOptions() const
{
    return _dbOptions.get();
}

void
Session::setResourceCache(ResourceCache* cache)
{
    _resourceCache = cache;
}

ResourceCache*
Session::getResourceCache()
{
    return _resourceCache.get();
}

MapFrame
Session::createMapFrame( Map::ModelParts parts ) const
{
    return MapFrame( _map.get(), parts );
}

void
Session::removeObject( const std::string& key )
{
    Threading::ScopedMutexLock lock( _objMapMutex );
    _objMap.erase( key );
}

void
Session::setStyles( StyleSheet* value )
{
    _styles = value ? value : new StyleSheet();
    _styleScriptEngine = 0L;

    // Create a script engine for the StyleSheet
    if (_styles)
    {
        if (_styles->script())
        {
            _styleScriptEngine = ScriptEngineFactory::create( Script(
                _styles->script()->code, 
                _styles->script()->language, 
                _styles->script()->name ) );
        }
        else
        {
            // If the stylesheet has no script set, create a default JS engine
            // This enables the use of "inline" scripting in StringExpression
            // and NumericExpression style values.
            _styleScriptEngine = ScriptEngineFactory::create("javascript", "", true);
        }
    }
}

ScriptEngine*
Session::getScriptEngine() const
{
    return _styleScriptEngine.get();
}

FeatureSource*
Session::getFeatureSource() const 
{ 
    return _featureSource.get(); 
}

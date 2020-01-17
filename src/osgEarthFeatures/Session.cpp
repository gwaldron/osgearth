/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthSymbology/StyleSheet>

#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

//---------------------------------------------------------------------------

Session::Session() :
osg::Object(),
_map(0L),
_mapInfo(0L)
{
    init();
}

Session::Session(const Map* map) :
osg::Object(),
_map(map),
_mapInfo(map)
{
    init();
}

Session::Session(const Map* map, StyleSheet* styles) :
osg::Object(),
_map(map),
_mapInfo(map),
_styles(styles)
{
    init();
}

Session::Session(const Map* map, StyleSheet* styles, FeatureSource* source, const osgDB::Options* dbOptions) :
osg::Object(),
_map(map),
_mapInfo(map),
_styles(styles),
_featureSource(source),
_dbOptions(dbOptions)
{
    init();
}

Session::Session(const Session& rhs, const osg::CopyOp& op) :
osg::Object(rhs, op),
_map(rhs._map.get()),
_mapInfo(rhs._mapInfo)
{
    //nop
}


void
Session::init()
{
    setStyles(_styles.get());

    // A new cache to optimize state changes. Since the cache lives in the Session, any
    // geometry created under this session takes advantage of it. That's reasonable since
    // tiles in a particular "layer" will tend to share state.
    _stateSetCache = new StateSetCache();

    _name = "Session (unnamed)";
}

Session::~Session()
{
    //nop
}

const osgDB::Options*
Session::getDBOptions() const
{
    // local options if they were set:
    if (_dbOptions.valid())
        return _dbOptions.get();

    // otherwise get them from the map if possible:
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
        return map->getReadOptions();

    return 0L;
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

osg::ref_ptr<const Map>
Session::getMap() const
{
    osg::ref_ptr<const Map> map;
    _map.lock(map);
    return map;
}

const SpatialReference*
Session::getMapSRS() const
{
    return _mapInfo.getSRS();
}

StateSetCache*
Session::getStateSetCache()
{
    return _stateSetCache.get();
}

void
Session::setStyles( StyleSheet* value )
{
    _styles = value ? value : new StyleSheet();
    initScriptEngine();
}

void
Session::initScriptEngine()
{
    _styleScriptEngine = 0L;

    // Create a script engine for the StyleSheet
    if (_styles)
    {
        if (_styles->script())
        {
            _styleScriptEngine = ScriptEngineFactory::createWithProfile(
                Script(
                    _styles->script()->code, 
                    _styles->script()->language, 
                    _styles->script()->name ),
                _styles->script()->profile );
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

void
Session::setFeatureSource(FeatureSource* fs)
{
    _featureSource = fs;
}

FeatureSource*
Session::getFeatureSource() const 
{ 
    return _featureSource.get(); 
}

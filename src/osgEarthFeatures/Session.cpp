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

Session::Session( const Map* map, StyleSheet* styles ) :
osg::Referenced( true ),
_map           ( map ),
_mapInfo       ( map )
//_resourceCache ( new ResourceCache(true) ) // make is thread-safe.
{
    if ( styles )
        setStyles( styles );
    else
        _styles = new StyleSheet();
}

#if 0
void
Session::setReferenceURI( const std::string& referenceURI )
{
    _referenceURI = referenceURI;
}

std::string
Session::resolveURI( const std::string& inputURI ) const
{
    return osgEarth::getFullPath( _referenceURI, inputURI );
}
#endif

MapFrame
Session::createMapFrame( Map::ModelParts parts ) const
{
    return MapFrame( _map.get(), parts );
}

void
Session::putObject( const std::string& key, osg::Referenced* object )
{
    //if ( dynamic_cast<osg::Node*>( object ) )
    //{
    //    OE_INFO << LC << "*** usage warning: storing an osg::Node in the Session cache is bad news;"
    //        << " live graph iterators can be invalidated." 
    //        << std::endl;
    //}

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
}

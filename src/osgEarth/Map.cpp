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

#include <osgEarth/Map>
#include <osgEarth/GeocentricMap>
#include <osgEarth/ProjectedMap>

using namespace osgEarth;

Map::Map(const MapConfig& mapConfig)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( MapEngine::s_mapEngineCacheMutex );

    const osgDB::ReaderWriter::Options* global_options = mapConfig.getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    // transcribe proxy settings:
    if ( !mapConfig.getProxyHost().empty() )
    {
        if ( !local_options.valid() )
            local_options = new osgDB::ReaderWriter::Options();

        std::stringstream buf;
        buf << local_options->getOptionString() << " "
            << "OSG_CURL_PROXY=" << mapConfig.getProxyHost() << " "
            << "OSG_CURL_PROXYPORT=" << mapConfig.getProxyPort();
        local_options->setOptionString( buf.str() );
    }

    osg::notify(osg::INFO) 
        << "[osgEarth] Map: options string = " 
        << (local_options.valid()? local_options->getOptionString() : "<empty>")
        << std::endl;

    
    MapConfig newMapConfig( mapConfig );
    newMapConfig.setGlobalOptions( local_options.get() );  

    if (mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC || 
        mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE )
    {     
        _engine = new GeocentricMap( newMapConfig );
    }
    else // if (mapConfig->getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED)
    {
        _engine = new ProjectedMap( newMapConfig );
    }

    addChild( _engine->initialize() );
}

Map::~Map()
{
    //NOP
}


MapEngine*
Map::getEngine() const
{
    return _engine.get();
}

const Profile*
Map::getProfile() const 
{
    return _engine->getProfile();
}

bool
Map::isOK() const
{
    return _engine->isOK();
}

void
Map::dirtyLayers()
{
    _engine->dirtyLayers();
}

void
Map::addLayer( Layer* layer )
{
    _engine->addLayer( layer );
}

void
Map::removeLayer( Layer* layer )
{
    _engine->removeLayer( layer );
}

void
Map::moveLayer( Layer* layer, int position )
{
    _engine->moveLayer( layer, position );
}

unsigned int
Map::getNumLayers() const
{
    return _engine->getNumLayers();
}

Layer*
Map::getLayer( unsigned int i ) const
{
    return _engine->getLayer( i );
}

void 
Map::getImageLayers( ImageLayerList& layers ) const
{
    _engine->getImageLayers( layers );
}

void 
Map::getElevationLayers( ElevationLayerList& layers ) const
{
    _engine->getElevationLayers( layers );
}

TileSource* 
Map::createTileSource( const SourceConfig& sourceConfig )
{
    return _engine->createTileSource( sourceConfig );
}


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

#include <osgEarth/MapEngineProperties>
#include <osg/Notify>

using namespace osgEarth;

LoadingPolicy::LoadingPolicy() :
_mode( MODE_STANDARD ),
_numThreads( 2 ),
_numThreadsPerCore( 2 ),
_numTileGenThreads( 4 )
{
    //nop
}

LoadingPolicy::LoadingPolicy( const LoadingPolicy::Mode& mode ) :
_mode( mode ),
_numThreads( 2 ),
_numThreadsPerCore( 2 ),
_numTileGenThreads( 4 )
{
    //nop
}

//----------------------------------------------------------------------------

ProxySettings::ProxySettings() {
    //nop
}

ProxySettings::ProxySettings( const std::string& host, int port ) :
_hostName(host),
_port(port)
{
    //nop
}

//----------------------------------------------------------------------------

MapEngineProperties::MapEngineProperties() :
_loadingPolicy( LoadingPolicy() ),
_verticalScale( 1.0f ),
_heightFieldSkirtRatio( 0.05f ),
_heightFieldSampleRatio( 1.0f ),
_proxySettings( ProxySettings() ),
_cacheOnly( false ),
_minTileRangeFactor( 6.0 ),
_normalizeEdges( false ),
_combineLayers( true ),
_maxLOD( 23 ),
_layeringTechnique( LAYERING_MULTITEXTURE ),
_enableLighting( true )
{
    //nop
}

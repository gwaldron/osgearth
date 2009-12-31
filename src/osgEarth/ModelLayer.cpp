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
#include <osgEarth/ModelLayer>
#include <osgEarth/Map>

using namespace osgEarth;

#define ATTR_MIN_RANGE "min_range"
#define ATTR_MAX_RANGE "max_range"

ModelLayer::ModelLayer( const std::string& name, const std::string& driver, const Config& driverConf ) :
osg::Referenced( true ),
_name( name ),
_driver( driver ),
_driverConf( driverConf ),
_minRange( 0 ),
_maxRange( FLT_MAX )
{
    if ( driverConf.hasValue( ATTR_MIN_RANGE ) )
        _minRange = driverConf.value<float>( ATTR_MIN_RANGE, 0.0f );
    if ( driverConf.hasValue( ATTR_MAX_RANGE ) )
        _maxRange = driverConf.value<float>( ATTR_MAX_RANGE, FLT_MAX );
}

ModelLayer::ModelLayer( const std::string& name, ModelSource* source ) :
osg::Referenced( true ),
_name( name ),
_modelSource( source ),
_minRange( 0 ),
_maxRange( FLT_MAX )
{
    //NOP
}

void
ModelLayer::initialize( const std::string& referenceURI, const Map* map )
{
    _referenceURI = referenceURI;

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _name, _driver, _driverConf );
    }

    if ( _modelSource.valid() )
    {
        _modelSource->initialize( _referenceURI, map );
    }
}

osg::Node*
ModelLayer::createNode( ProgressCallback* progress )
{
    osg::Node* result = 0L;

    if ( _modelSource.valid() )
    {
        result = _modelSource->createNode( progress );

        if ( _minRange.isSet() || _maxRange.isSet() )
        {
            osg::LOD* lod = new osg::LOD();
            lod->addChild( result, _minRange.get(), _maxRange.get() );
            result = lod;
        }
    }

    return result;
}

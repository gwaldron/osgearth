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

ModelLayer::ModelLayer( const std::string& name, const std::string& driver, const Config& driverConf ) :
osg::Referenced( true ),
_name( name ),
_driver( driver ),
_driverConf( driverConf )
{
    //nop
}

ModelLayer::ModelLayer( const std::string& name, ModelSource* source ) :
osg::Referenced( true ),
_name( name ),
_modelSource( source )
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
    }

    return result;
}

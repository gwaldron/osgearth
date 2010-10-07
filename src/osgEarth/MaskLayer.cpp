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
#include <osgEarth/MaskLayer>
#include <osgEarth/Map>

using namespace osgEarth;

#define MASK_MODEL_DRIVER "feature_stencil"

MaskLayer::MaskLayer( const ConfigOptions& options )
{
    // just in case the caller did not ref the parameter:
    //osg::ref_ptr<const DriverOptions> tempHold = options;

    // copy the input options and set some special settings:
    Config conf = options.getConfig();
    conf.update( "mask", "true" );
    conf.update( "inverted", "true" );
    conf.update( "extrusion_distance", "100000" );
    _driverOptions = DriverConfigOptions( conf );
    _driverOptions.setDriver( MASK_MODEL_DRIVER );
}

void
MaskLayer::initialize( const std::string& referenceURI, const Map* map )
{
    _referenceURI = referenceURI;

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _driverOptions );
    }

    if ( _modelSource.valid() )
    {
        _modelSource->initialize( _referenceURI, map );
    }
}

osg::Node*
MaskLayer::getOrCreateNode( ProgressCallback* progress )
{
    osg::Node* result = 0L;

    if ( _modelSource.valid() )
    {
        result = _modelSource->createNode( progress );

        // a hack to immediately update-traverse this node so it can generate its 
        // MaskNodes. Otherwise MapNode will not be able to apply it.
        if ( result )
        {
            osg::NodeVisitor nv( osg::NodeVisitor::UPDATE_VISITOR, osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
            result->accept( nv );
        }
    }

    return result;
}

Config
MaskLayer::getConfig() const
{
    Config conf = _driverOptions.getConfig();
    conf.key() = "mask";
    conf.remove( "driver" );
    return conf;
}

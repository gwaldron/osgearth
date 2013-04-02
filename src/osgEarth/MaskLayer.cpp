/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#define LC "[MaskLayer] "

using namespace osgEarth;
//------------------------------------------------------------------------

MaskLayerOptions::MaskLayerOptions( const ConfigOptions& options ) :
ConfigOptions( options )
{
    setDefaults();
    fromConfig( _conf ); 
}

MaskLayerOptions::MaskLayerOptions( const std::string& name, const MaskSourceOptions& driverOptions ) :
ConfigOptions()
{
    setDefaults();
    fromConfig( _conf );
    _name = name;
    _driver = driverOptions;
}

void
MaskLayerOptions::setDefaults()
{
    //nop
}

Config
MaskLayerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    conf.updateIfSet( "name", _name );

    return conf;
}

void
MaskLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name", _name );
}

void
MaskLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

MaskLayer::MaskLayer( const MaskLayerOptions& options ) :
_initOptions( options )
{
    copyOptions();
}

MaskLayer::MaskLayer( const std::string& name, const MaskSourceOptions& options ) :
_initOptions( MaskLayerOptions( name, options ) )
{
    copyOptions();
}

MaskLayer::MaskLayer( const MaskLayerOptions& options, MaskSource* source ) :
_maskSource( source ),
_initOptions( options )
{
    copyOptions();
}

void
MaskLayer::copyOptions()
{
    _runtimeOptions = _initOptions;
}

void
MaskLayer::initialize( const osgDB::Options* dbOptions, const Map* map )
{
    _dbOptions = osg::clone(dbOptions);

    if ( !_maskSource.valid() && _initOptions.driver().isSet() )
    {
        _maskSource = MaskSourceFactory::create( *_initOptions.driver() );
    }

    if ( _maskSource.valid() )
    {
        _maskSource->initialize( dbOptions, map );
    }
}

osg::Vec3dArray*
MaskLayer::getOrCreateBoundary( float heightScale, const SpatialReference *srs, ProgressCallback* progress )
{
    if ( _maskSource.valid() )
    {
        // if the model source has changed, regenerate the node.
        if ( _boundary.valid() && !_maskSource->inSyncWith(_maskSourceRev) )
        {
            _boundary = 0L;
        }

        if ( !_boundary.valid() )
        {
			_boundary = _maskSource->createBoundary( srs, progress );

			for (osg::Vec3dArray::iterator vIt = _boundary->begin(); vIt != _boundary->end(); ++vIt)
				vIt->z() = vIt->z() * heightScale;

            _maskSource->sync( _maskSourceRev );
        }
    }

    return _boundary.get();
}

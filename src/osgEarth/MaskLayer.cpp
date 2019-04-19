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
#include <osgEarth/MaskLayer>

#define LC "[MaskLayer] "

using namespace osgEarth;

//------------------------------------------------------------------------

MaskLayerOptions::MaskLayerOptions(const ConfigOptions& options) :
LayerOptions(options),
_minLevel( 0 )
{
    setDefaults();
    fromConfig( _conf ); 
}

void
MaskLayerOptions::setDefaults()
{
    _minLevel.init( 0 );
}

Config
MaskLayerOptions::getConfig() const
{
    Config conf = LayerOptions::getConfig();
    conf.key() = "mask";
    conf.set( "min_level", _minLevel );
    return conf;
}

void
MaskLayerOptions::fromConfig( const Config& conf )
{
    conf.get( "min_level", _minLevel );
}

void
MaskLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

//MaskLayer::MaskLayer(const MaskLayerOptions& options) :
//Layer(&_optionsConcrete),
//_options(&_optionsConcrete),
//_optionsConcrete(options)
//{
//    init();
//}

//MaskLayer::MaskLayer(const MaskLayerOptions& options, MaskSource* source) :
//Layer(&_optionsConcrete),
//_options(&_optionsConcrete),
//_optionsConcrete(options),
//_maskSource(source)
//{
//    init();
//}

MaskLayer::MaskLayer(MaskLayerOptions* optionsPtr) :
Layer(optionsPtr),
_options(optionsPtr)
{
    //nop - init will be called by base class
}

//const Status&
//MaskLayer::open()
//{
//    if (!_maskSource.valid() && !options().driver().isSet())
//    {
//        return setStatus(Status::Error(Status::ConfigurationError, "Missing data source for mask geometry"));
//    }
//
//    if (!_maskSource.valid() && options().driver().isSet())
//    {
//        _maskSource = MaskSourceFactory::create(options());
//        if (!_maskSource.valid())
//        {
//            return setStatus(Status::Error(Status::ServiceUnavailable, 
//                Stringify() << "Failed to load mask driver" << options().name()));
//        }
//    }
//
//    return setStatus(_maskSource->open(getReadOptions()));
//}

//osg::Vec3dArray*
//MaskLayer::getOrCreateMaskBoundary( float heightScale, const SpatialReference *srs, ProgressCallback* progress )
//{
//    if (getStatus().isError())
//    {
//        return 0L;
//    }
//
//    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
//    if ( _maskSource.valid() )
//    {
//        // if the model source has changed, regenerate the node.
//        if ( _boundary.valid() && !_maskSource->inSyncWith(_maskSourceRev) )
//        {
//            _boundary = 0L;
//        }
//
//        if ( !_boundary.valid() )
//        {
//			_boundary = _maskSource->createBoundary( srs, progress );
//            
//            if (_boundary.valid())
//            {
//			    for (osg::Vec3dArray::iterator vIt = _boundary->begin(); vIt != _boundary->end(); ++vIt)
//    				vIt->z() = vIt->z() * heightScale;
//
//                _maskSource->sync( _maskSourceRev );
//            }
//        }
//    }
//
//    if (!_boundary.valid())
//    {
//        setStatus(Status::Error("Failed to create masking boundary"));
//    }
//
//    return _boundary.get();
//}

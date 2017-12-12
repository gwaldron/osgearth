/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

/****************************************************************/


ModelSourceOptions::ModelSourceOptions( const ConfigOptions& options ) :
DriverConfigOptions( options ),
_minRange          ( 0.0f ),
_maxRange          ( FLT_MAX ),
_renderOrder       ( 11 ),
_renderBin         ( "DepthSortedBin" ),
_depthTestEnabled  ( true )
{ 
    fromConfig(_conf);
}

ModelSourceOptions::~ModelSourceOptions()
{
    //nop
}

void
ModelSourceOptions::fromConfig( const Config& conf )
{
    conf.getIfSet<float>( "min_range", _minRange );
    conf.getIfSet<float>( "max_range", _maxRange );
    conf.getIfSet<int>( "render_order", _renderOrder );
    conf.getIfSet("render_bin", _renderBin );
    conf.getIfSet<bool>( "depth_test_enabled", _depthTestEnabled );
}

void
ModelSourceOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
ModelSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.set( "min_range", _minRange );
    conf.set( "max_range", _maxRange );
    conf.set( "render_order", _renderOrder );
    conf.set( "render_bin", _renderBin );
    conf.set( "depth_test_enabled", _depthTestEnabled );
    return conf;
}

//------------------------------------------------------------------------

ModelSource::ModelSource( const ModelSourceOptions& options ) :
_options( options )
{
    _sgCallbacks = new SceneGraphCallbacks();
}

ModelSource::~ModelSource()
{
   //nop
}

const Status&
ModelSource::open(const osgDB::Options* readOptions)
{
    _status = initialize(readOptions);
    return _status;
}

osg::Node* 
ModelSource::createNode(const Map*        map,
                        ProgressCallback* progress )
{
    if (getStatus().isError())
    {
        return 0L;
    }

    osg::Node* node = createNodeImplementation(map, progress);
    if ( node )
    {
        getSceneGraphCallbacks()->firePreMergeNode(node);
        getSceneGraphCallbacks()->firePostMergeNode(node);
    }
    return node;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[ModelSourceFactory] "
#define MODEL_SOURCE_OPTIONS_TAG "__osgEarth::ModelSourceOptions"

ModelSourceFactory::~ModelSourceFactory()
{
    //nop
}

ModelSource*
ModelSourceFactory::create( const ModelSourceOptions& options )
{
    osg::ref_ptr<ModelSource> source;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_model_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( MODEL_SOURCE_OPTIONS_TAG, (void*)&options );

        osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt, rwopts.get() );
        source = dynamic_cast<ModelSource*>( object.release() );
    }
    else
    {
        OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return source.release();
}

//------------------------------------------------------------------------

const ModelSourceOptions&
ModelSourceDriver::getModelSourceOptions( const osgDB::ReaderWriter::Options* options ) const
{
    static ModelSourceOptions s_default;
    const void* data = options->getPluginData(MODEL_SOURCE_OPTIONS_TAG);
    return data ? *static_cast<const ModelSourceOptions*>(data) : s_default;
}

ModelSourceDriver::~ModelSourceDriver()
{
}

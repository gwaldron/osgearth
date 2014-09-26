/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
_depthTestEnabled  ( true )
{ 
    fromConfig(_conf);
}

ModelSourceOptions::~ModelSourceOptions()
{
}

void
ModelSourceOptions::fromConfig( const Config& conf )
{
    conf.getIfSet<float>( "min_range", _minRange );
    conf.getIfSet<float>( "max_range", _maxRange );
    conf.getIfSet<int>( "render_order", _renderOrder );
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
    conf.updateIfSet( "min_range", _minRange );
    conf.updateIfSet( "max_range", _maxRange );
    conf.updateIfSet( "render_order", _renderOrder );
    conf.updateIfSet( "depth_test_enabled", _depthTestEnabled );
    return conf;
}

//------------------------------------------------------------------------

ModelSource::ModelSource( const ModelSourceOptions& options ) :
_options( options )
{
   _preMergeOps  = new RefNodeOperationVector();
   _postMergeOps = new RefNodeOperationVector();
}

ModelSource::~ModelSource()
{
   //nop
}


osg::Node* 
ModelSource::createNode(const Map*            map,
                        const osgDB::Options* dbOptions,
                        ProgressCallback*     progress )
{
    osg::Node* node = createNodeImplementation(map, dbOptions, progress);
    if ( node )
    {
        firePostProcessors( node );
    }
    return node;
}


void 
ModelSource::addPreMergeOperation( NodeOperation* op )
{
    if ( op )
    {
        _preMergeOps->mutex().writeLock();
        _preMergeOps->push_back( op );
        _preMergeOps->mutex().writeUnlock();
    }
}


void
ModelSource::removePreMergeOperation( NodeOperation* op )
{
    if ( op )
    {
        _preMergeOps->mutex().writeLock();
        NodeOperationVector::iterator i = std::find( _preMergeOps->begin(), _preMergeOps->end(), op );
        if ( i != _postMergeOps->end() )
            _preMergeOps->erase( i );
        _preMergeOps->mutex().writeUnlock();
    }
}


void 
ModelSource::addPostMergeOperation( NodeOperation* op )
{
    if ( op )
    {
        _postMergeOps->mutex().writeLock();
        _postMergeOps->push_back( op );
        _postMergeOps->mutex().writeUnlock();
    }
}


void
ModelSource::removePostMergeOperation( NodeOperation* op )
{
    if ( op )
    {
        _postMergeOps->mutex().writeLock();
        NodeOperationVector::iterator i = std::find( _postMergeOps->begin(), _postMergeOps->end(), op );
        if ( i != _postMergeOps->end() )
            _postMergeOps->erase( i );
        _postMergeOps->mutex().writeUnlock();
    }
}


void
ModelSource::firePostProcessors( osg::Node* node )
{
    if ( node )
    {
        // pres:
        _preMergeOps->mutex().readLock();
        for( NodeOperationVector::iterator i = _preMergeOps->begin(); i != _preMergeOps->end(); ++i )
        {
            i->get()->operator()( node );
        }
        _preMergeOps->mutex().readUnlock();

        // posts:
        _postMergeOps->mutex().readLock();
        for( NodeOperationVector::iterator i = _postMergeOps->begin(); i != _postMergeOps->end(); ++i )
        {
            i->get()->operator()( node );
        }
        _postMergeOps->mutex().readUnlock();
    }
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[ModelSourceFactory] "
#define MODEL_SOURCE_OPTIONS_TAG "__osgEarth::ModelSourceOptions"

ModelSourceFactory::~ModelSourceFactory()
{
}

ModelSource*
ModelSourceFactory::create( const ModelSourceOptions& options )
{
    ModelSource* modelSource = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_model_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( MODEL_SOURCE_OPTIONS_TAG, (void*)&options );

        modelSource = dynamic_cast<ModelSource*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
        if ( !modelSource )
        {
            OE_WARN << "FAILED to load model source driver \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return modelSource;
}

//------------------------------------------------------------------------

const ModelSourceOptions&
ModelSourceDriver::getModelSourceOptions( const osgDB::ReaderWriter::Options* options ) const
{
    return *static_cast<const ModelSourceOptions*>( options->getPluginData( MODEL_SOURCE_OPTIONS_TAG ) );
}

ModelSourceDriver::~ModelSourceDriver()
{
}

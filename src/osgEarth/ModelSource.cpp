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
#include <osgEarth/ModelSource>
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

/****************************************************************/

ModelSourceOptions::ModelSourceOptions( const PluginOptions* po ) :
DriverOptions( po ),
_minRange(0),
_maxRange(FLT_MAX),
_renderOrder( 11 )
{
    config().getIfSet<float>( "min_range", _minRange );
    config().getIfSet<float>( "max_range", _maxRange );
    config().getIfSet<int>( "render_order", _renderOrder );
}

Config
ModelSourceOptions::toConfig() const
{
    Config conf = DriverOptions::toConfig();
    conf.updateIfSet( "min_range", _minRange );
    conf.updateIfSet( "max_range", _maxRange );
    conf.updateIfSet( "render_order", _renderOrder );
    return conf;
}

/****************************************************************/

ModelSource::ModelSource( const PluginOptions* options )
{
    this->setThreadSafeRefUnref( true );

    _options = dynamic_cast<const ModelSourceOptions*>( options );
    if ( !_options.valid() )
        _options = new ModelSourceOptions( options );
}

/****************************************************************/

ModelSource*
ModelSourceFactory::create( const DriverOptions* driverOptions )
{
    ModelSource* modelSource = 0L;
    if ( driverOptions )
    {
        std::string driverExt = std::string(".osgearth_model_") + driverOptions->driver();

        modelSource = dynamic_cast<ModelSource*>( osgDB::readObjectFile( driverExt, driverOptions ) );
        if ( modelSource )
        {
            modelSource->setName( driverOptions->name() );
        }
        else
        {
            osg::notify(osg::NOTICE)
                << "[osgEarth] WARNING: Failed to load model source driver for " << driverExt << std::endl;
        }
    }
    else
    {
        osg::notify(osg::NOTICE)
            << "[osgEarth] ERROR: null driver options to ModelSourceFactory" << std::endl;
    }

    return modelSource;
}


// to be deprecated.
ModelSource*
ModelSourceFactory::create(const std::string& name,
                           const std::string& driver,
                           const Config&      driverConf,
                           const osgDB::ReaderWriter::Options* globalOptions )
{
    osg::ref_ptr<PluginOptions> options = globalOptions?
        new PluginOptions( *globalOptions ) :
        new PluginOptions();

    //Setup the plugin options for the source
    options->config() = driverConf;

    osg::notify(osg::INFO)
        << "[osgEarth] Model Driver " << driver << ", config =" << std::endl << driverConf.toString() << std::endl;

	// Load the source from a plugin.
    osg::ref_ptr<ModelSource> source = dynamic_cast<ModelSource*>(
                osgDB::readObjectFile( std::string(".osgearth_model_") + driver, options.get()));

    if ( source.valid() )
    {
        source->setName( name );
    }
    else
	{
		osg::notify(osg::NOTICE) << "[osgEarth] Warning: Could not load Model driver "  << driver << std::endl;
	}

	return source.release();
}


ModelSource*
ModelSourceFactory::create(const Config& sourceConf,
                           const osgDB::ReaderWriter::Options* globalOptions )
{
    return create(
        sourceConf.attr( "name" ),
        sourceConf.attr( "driver" ),
        sourceConf,
        globalOptions );
}


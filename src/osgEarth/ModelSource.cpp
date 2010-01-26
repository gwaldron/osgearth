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

#define ATTR_MIN_RANGE "min_range"
#define ATTR_MAX_RANGE "max_range"


ModelSource::ModelSource( const PluginOptions* options ) :
_options( options ),
_minRange( 0.0f ),
_maxRange( FLT_MAX )
{
    this->setThreadSafeRefUnref( true );
    if ( options )
    {
        const Config& conf = options->config();

        _name = conf.value( "name" );
        
        if ( conf.hasValue( ATTR_MIN_RANGE ) )
            _minRange = conf.value<float>( ATTR_MIN_RANGE, 0.0f );
        if ( conf.hasValue( ATTR_MAX_RANGE ) )
            _maxRange = conf.value<float>( ATTR_MAX_RANGE, FLT_MAX );
    }
}

const PluginOptions* 
ModelSource::getOptions() const
{
    return _options.get();
}


/****************************************************************************/

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


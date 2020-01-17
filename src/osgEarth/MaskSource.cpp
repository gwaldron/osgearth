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
#include <osgEarth/MaskSource>
#include <osgEarth/Registry>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace OpenThreads;

/****************************************************************/

MaskSourceOptions::~MaskSourceOptions()
{
    //nop
}

void
MaskSourceOptions::fromConfig( const Config& conf )
{
    //nop
}

void
MaskSourceOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
MaskSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    return conf;
}

//------------------------------------------------------------------------

MaskSource::MaskSource( const MaskSourceOptions& options ) :
_options( options )
{
    //nop
}

MaskSource::~MaskSource()
{
    //nop
}

const Status&
MaskSource::open(const osgDB::Options* readOptions)
{
    _status = initialize(readOptions);
    return _status;
}

//------------------------------------------------------------------------

MaskSourceDriver::~MaskSourceDriver()
{
    //nop
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[MaskSourceFactory] "
#define MASK_SOURCE_OPTIONS_TAG "__osgEarth::MaskSourceOptions"

MaskSourceFactory::~MaskSourceFactory()
{
    //nop
}

MaskSource*
MaskSourceFactory::create( const MaskSourceOptions& options )
{
    osg::ref_ptr<MaskSource> source;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_mask_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( MASK_SOURCE_OPTIONS_TAG, (void*)&options );

        osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt, rwopts.get() );
        source = dynamic_cast<MaskSource*>( object.release() );
        if ( source )
        {
            OE_INFO << "Loaded MaskSource driver \"" << options.getDriver() << "\" OK" << std::endl;
        }
        else
        {
            OE_WARN << "FAIL, unable to load MaskSource driver for \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return source.release();
}

//------------------------------------------------------------------------

const MaskSourceOptions&
MaskSourceDriver::getMaskSourceOptions( const osgDB::ReaderWriter::Options* options ) const
{
    static MaskSourceOptions s_default;
    const void* data = options->getPluginData(MASK_SOURCE_OPTIONS_TAG);
    return data ? *static_cast<const MaskSourceOptions*>(data) : s_default;
}

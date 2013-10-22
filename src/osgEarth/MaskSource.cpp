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
#include <osgEarth/MaskSource>
#include <osgEarth/Registry>
#include <osg/Notify>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace OpenThreads;

/****************************************************************/

MaskSourceOptions::~MaskSourceOptions()
{
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
    //TODO: is this really necessary?
    this->setThreadSafeRefUnref( true );
}

MaskSource::~MaskSource()
{
}

//------------------------------------------------------------------------

MaskSourceDriver::~MaskSourceDriver()
{
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[MaskSourceFactory] "
#define MASK_SOURCE_OPTIONS_TAG "__osgEarth::MaskSourceOptions"

MaskSourceFactory::~MaskSourceFactory()
{
}

MaskSource*
MaskSourceFactory::create( const MaskSourceOptions& options )
{
    MaskSource* source = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_mask_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( MASK_SOURCE_OPTIONS_TAG, (void*)&options );

        source = dynamic_cast<MaskSource*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
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

    return source;
}

//------------------------------------------------------------------------

const MaskSourceOptions&
MaskSourceDriver::getMaskSourceOptions( const osgDB::ReaderWriter::Options* options ) const
{
    return *static_cast<const MaskSourceOptions*>( options->getPluginData( MASK_SOURCE_OPTIONS_TAG ) );
}

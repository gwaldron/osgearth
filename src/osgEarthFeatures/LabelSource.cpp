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
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/Registry>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[LabelSource] "


//------------------------------------------------------------------------

LabelSourceOptions::~LabelSourceOptions()
{
}

void
LabelSourceOptions::fromConfig( const Config& conf )
{
    //nop
}

void
LabelSourceOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
LabelSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    return conf;
}

//------------------------------------------------------------------------
LabelSource::~LabelSource()
{
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[LabeSourceFactory] "
#define LABEL_SOURCE_OPTIONS_TAG "__osgEarth::Features::LabelSourceOptions"

LabelSource*
LabelSourceFactory::create( const LabelSourceOptions& options )
{
    LabelSource* labelSource = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_label_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( LABEL_SOURCE_OPTIONS_TAG, (void*)&options );

        labelSource = dynamic_cast<LabelSource*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
        if ( labelSource )
        {
            //modelSource->setName( options.getName() );
            //OE_INFO << "Loaded LabelSource driver \"" << options.getDriver() << "\" OK" << std::endl;
        }
        else
        {
            OE_WARN << "FAIL, unable to load label source driver for \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return labelSource;
}

//------------------------------------------------------------------------

const LabelSourceOptions&
LabelSourceDriver::getLabelSourceOptions( const osgDB::ReaderWriter::Options* options ) const
{
    static LabelSourceOptions s_default;
    const void* data = options->getPluginData(LABEL_SOURCE_OPTIONS_TAG);
    return data ? *static_cast<const LabelSourceOptions*>(data) : s_default;
}

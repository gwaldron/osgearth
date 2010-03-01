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
#include <osgEarthFeatures/FeatureRasterizer>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Features;

FeatureRasterizer::FeatureRasterizer( const osgDB::ReaderWriter::Options* options ) :
_options( options )
{
    //NOP
}


FeatureRasterizer*
FeatureRasterizerFactory::create(const std::string& driver,
                                 const Config& driverConf,
                                 const osgDB::ReaderWriter::Options* globalOptions )
{
    osg::ref_ptr<PluginOptions> pluginOptions = globalOptions?
        new PluginOptions( *globalOptions ) :
        new PluginOptions();

    // Setup the plugin options for the source
    pluginOptions->config() = driverConf;

	//Load the source from the a plugin.  The "." prefix causes OSG to select the correct plugin.
    //For instance, the WMS plugin can be loaded by using ".osgearth_wms" as the filename
    osg::ref_ptr<FeatureRasterizer> rasterizer = dynamic_cast<FeatureRasterizer*>(
        osgDB::readObjectFile( ".osgearth_rasterizer_" + driver, pluginOptions.get()));

	if ( !rasterizer.valid() )
	{
		osg::notify(osg::NOTICE) << "[osgEarth] Warning: Could not load Rasterizer for driver "  << driver << std::endl;
	}

	return rasterizer.release();
}


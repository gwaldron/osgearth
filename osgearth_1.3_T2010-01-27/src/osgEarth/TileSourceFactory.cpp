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

#include <osgEarth/TileSourceFactory>
#include <osgEarth/Caching>
#include <osgEarth/Registry>

using namespace osgEarth;

TileSource*
TileSourceFactory::create( const std::string& driver,
                           const Config&      driverConf,
						   const osgDB::ReaderWriter::Options* global_options
                           )
{
    osg::ref_ptr<PluginOptions> options = global_options?
        new PluginOptions( *global_options ) :
        new PluginOptions();

    //Setup the plugin options for the source
    options->config() = driverConf;

    osg::notify(osg::INFO)
        << "[osgEarth] Driver " << driver << ", config =" << std::endl << driverConf.toString() << std::endl;

	//Load the source from the a plugin.  The "." prefix causes OSG to select the correct plugin.
    //For instance, the WMS plugin can be loaded by using ".osgearth_wms" as the filename
    osg::ref_ptr<TileSource> tileSource = dynamic_cast<TileSource*>(
                osgDB::readObjectFile( std::string(".osgearth_") + driver, options.get()));

	if (!tileSource.valid())
	{
		osg::notify(osg::NOTICE) << "[osgEarth] Warning: Could not load TileSource for driver "  << driver << std::endl;
	}

	return tileSource.release();
}

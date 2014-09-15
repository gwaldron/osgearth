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
#include <osgEarth/Extension>
#include <osgEarth/Registry>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

using namespace osgEarth;

#define LC "[Extension] "

#define EXTENSION_OPTIONS_TAG "__osgEarth::ExtensionOptions"


Extension*
Extension::create(const std::string& name, const ConfigOptions& options)
{
    osg::ref_ptr<Extension> extension;

    if ( name.empty() )
    {
        OE_WARN << LC << "ILLEGAL- no driver set for tile source" << std::endl;
        return 0L;
    }

    // convey the configuration options:
    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData( EXTENSION_OPTIONS_TAG, (void*)&options );

    std::string pluginExtension = std::string( ".osgearth_" ) + name;

    osg::ref_ptr<osg::Object> result = osgDB::readObjectFile( pluginExtension, dbopt.get() );
    if ( !result.valid() )
    {
        // be quiet.
        //OE_WARN << LC << "Failed to load Extension \"" << name << "\"" << std::endl;
        return 0L;
    }

    // make sure it's actually an Extension:
    extension = dynamic_cast<Extension*>( result.release() );
    if ( !extension )
    {
        OE_WARN << LC << "Plugin \"" << name << "\" is not an Extension" << std::endl;
        return 0L;
    }

    return extension.release();
}


const ConfigOptions&
Extension::getConfigOptions(const osgDB::Options* options)
{
    return *static_cast<const ConfigOptions*>(
        options->getPluginData( EXTENSION_OPTIONS_TAG ) );
}

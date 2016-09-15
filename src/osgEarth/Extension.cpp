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


Extension::Extension()
{
    //nop
}

const ConfigOptions&
Extension::getConfigOptions() const
{
    return _defaultOptions;
}

Extension*
Extension::create(const std::string& name, const ConfigOptions& options)
{
    if ( name.empty() )
    {
        OE_WARN << LC << "ILLEGAL- Extension::create requires a plugin name" << std::endl;
        return 0L;
    }

    // convey the configuration options:
    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData( EXTENSION_OPTIONS_TAG, (void*)&options );

    std::string pluginExtension = std::string( ".osgearth_" ) + name;

    // use this instead of osgDB::readObjectFile b/c the latter prints a warning msg.
    osgDB::ReaderWriter::ReadResult rr = osgDB::Registry::instance()->readObject( pluginExtension, dbopt.get() );
    if ( !rr.validObject() || rr.error() )
    {
        // quietly fail so we don't get tons of msgs.
        return 0L;
    }

    Extension* extension = dynamic_cast<Extension*>( rr.getObject() );
    if ( extension == 0L )
    {
        OE_WARN << LC << "Plugin \"" << name << "\" is not an Extension" << std::endl;
        return 0L;
    }

    // for automatic serialization, in the event that the subclass does not
    // implement getConfigOptions.
    extension->_defaultOptions = options;

    if (extension->getName().empty())
        extension->setName(name);

    rr.takeObject();
    return extension;
}


const ConfigOptions&
Extension::getConfigOptions(const osgDB::Options* options)
{
    static ConfigOptions s_default;
    const void* data = options->getPluginData(EXTENSION_OPTIONS_TAG);
    return data ? *static_cast<const ConfigOptions*>(data) : s_default;
}

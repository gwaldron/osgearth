/* osgEarth
 * Copyright 2008-2013 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Extension>
#include <osgEarth/Registry>

using namespace osgEarth;

#define LC "[Extension] "

#define EXTENSION_OPTIONS_TAG "__osgEarth::ExtensionOptions"


Extension::Extension()
{
    //nop
    _defaultOptions = Config("extension");
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

    std::string pluginExtension = std::string( "osgearth_" ) + name;

    // use this instead of osgDB::readObjectFile b/c the latter prints a warning msg.
    auto rw = osgDB::Registry::instance()->getReaderWriterForExtension(pluginExtension);
    if (!rw)
    {
        return nullptr;
    }

    auto rr = rw->readObject("." + pluginExtension, dbopt.get());
    if ( !rr.validObject() || rr.error() )
    {
        // quietly fail so we don't get tons of msgs.
        return nullptr;
    }

    Extension* extension = dynamic_cast<Extension*>( rr.getObject() );
    if ( extension == nullptr )
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

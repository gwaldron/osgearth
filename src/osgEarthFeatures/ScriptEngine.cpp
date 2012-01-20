/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarth/Notify>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Features;

/****************************************************************/

void
ScriptEngineOptions::fromConfig( const Config& conf )
{
    optional<std::string> val;
    if (conf.getIfSet<std::string>( "script_code", val))
    {
        Script cfgScript(val.get());
        
        if (conf.getIfSet<std::string>( "script_language", val ))
          cfgScript.setLanguage(val.get());

        if (conf.getIfSet<std::string>( "script_name", val ))
          cfgScript.setName(val.get());
    }
}

void
ScriptEngineOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
ScriptEngineOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();

    if (_script.isSet())
    {
      if (!_script->getCode().empty()) conf.update("script_code", _script->getCode());
      if (!_script->getLanguage().empty()) conf.update("script_language", _script->getLanguage());
      if (!_script->getName().empty()) conf.update("script_name", _script->getName());
    }

    return conf;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[ScriptEngineFactory] "
#define SCRIPT_ENGINE_OPTIONS_TAG "__osgEarth::Features::ScriptEngineOptions"

ScriptEngine*
ScriptEngineFactory::create( const std::string& language, const std::string& engineName )
{
  ScriptEngineOptions opts;
  opts.setDriver(language + (engineName.empty() ? "" : (std::string("_") + engineName)));

  return create(opts);
}

ScriptEngine*
ScriptEngineFactory::create( const Script& script, const std::string& engineName )
{
  ScriptEngineOptions opts;
  opts.setDriver(script.getLanguage() + (engineName.empty() ? "" : (std::string("_") + engineName)));
  opts.script() = script;

  return create(opts);
}

ScriptEngine*
ScriptEngineFactory::create( const ScriptEngineOptions& options )
{
    ScriptEngine* scriptEngine = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_scriptengine_") + options.getDriver();

        osg::ref_ptr<osgDB::ReaderWriter::Options> rwopts = new osgDB::ReaderWriter::Options();
        rwopts->setPluginData( SCRIPT_ENGINE_OPTIONS_TAG, (void*)&options );

        scriptEngine = dynamic_cast<ScriptEngine*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
        if ( scriptEngine )
        {
            OE_INFO << "Loaded ScriptEngine driver \"" << options.getDriver() << "\" OK" << std::endl;
        }
        else
        {
            OE_WARN << "FAIL, unable to load ScriptEngine driver for \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return scriptEngine;
}

//------------------------------------------------------------------------

const ScriptEngineOptions&
ScriptEngineDriver::getScriptEngineOptions( const osgDB::ReaderWriter::Options* options ) const
{
    return *static_cast<const ScriptEngineOptions*>( options->getPluginData( SCRIPT_ENGINE_OPTIONS_TAG ) );
}
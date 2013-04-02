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
#include <osgDB/ReaderWriter>
#include <osgEarthDrivers/script_engine_v8/JavascriptEngineV8>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarth/Common>
#include <osgEarth/Config>
#include <osgDB/FileNameUtils>


class JavascriptEngineV8Factory : public osgEarth::Features::ScriptEngineDriver
{
public:
    JavascriptEngineV8Factory()
    {
        supportsExtension( "osgearth_scriptengine_javascript", "osgEarth scriptengine javascript plugin" );
        supportsExtension( "osgearth_scriptengine_javascript_v8", "osgEarth scriptengine javascript V8 plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth ScriptEngine Javascript V8 Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const osgDB::ReaderWriter::Options* options) const
    {
      if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )) )
            return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

        return osgDB::ReaderWriter::ReadResult( new JavascriptEngineV8( getScriptEngineOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_scriptengine_javascript, JavascriptEngineV8Factory)

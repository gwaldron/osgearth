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
#include <osgEarth/ShaderLoader>
#include <osgEarth/URI>
#include <osgEarth/VirtualProgram>
#include <osgDB/FileUtils>

#undef  LC
#define LC "[ShaderLoader] "

using namespace osgEarth;


typedef std::map<std::string,std::string> StringMap;


// find the value of a pragma, e.g.:
//   #pragma oe_key value
// returns the string "value" (without the quotes).
std::string
ShaderLoader::getPragmaValue(const std::string& source, const std::string& key)
{
    std::string token("#pragma " + key);
    std::string::size_type statementPos = source.find(token);
    if ( statementPos == std::string::npos )
        return "";

    // no quotes; parse to newline.
    std::string::size_type startPos = source.find_first_not_of(" \t", statementPos+token.length());
    if ( startPos == std::string::npos )
        return ""; // no whitespace after the pragma key

    std::string::size_type newlinePos = source.find('\n', startPos);
    if ( newlinePos == std::string::npos )
        return ""; // new newline found after pragma

    return trim(source.substr(startPos, newlinePos-startPos));
}

void
ShaderLoader::getAllPragmaValues(const std::string&     source,
                                 const std::string&     key,
                                 std::set<std::string>& output)
{
    std::string token("#pragma " + key);
    std::string::size_type pragmaPos = 0;
    while( pragmaPos != std::string::npos )
    {
        pragmaPos = source.find(token, pragmaPos);
        if ( pragmaPos != std::string::npos )
        {
            std::string::size_type startPos = source.find_first_not_of(" \t", pragmaPos+token.length());
            if ( startPos != std::string::npos )
            {
                std::string::size_type newlinePos = source.find('\n', startPos);
                if ( newlinePos != std::string::npos )
                {
                    const size_t len = newlinePos - startPos;
                    if ( len > 0 )
                    {
                        output.insert( trim(source.substr(startPos, len)) );
                    }
                    pragmaPos = newlinePos;
                }
                else
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
    }
}

std::string
ShaderLoader::load(const std::string&    filename,
                   const ShaderPackage&  package,
                   const osgDB::Options* dbOptions)
{
    std::string output;
    bool useInlineSource = false;

    URIContext context( dbOptions );
    URI uri(filename, context);

    std::string inlineSource;
    ShaderPackage::SourceMap::const_iterator source = package._sources.find(filename);
    if ( source != package._sources.end() )
        inlineSource = source->second;

    std::string path = osgDB::findDataFile(uri.full(), dbOptions);
    if ( path.empty() )
    {
        output = inlineSource;
        useInlineSource = true;
        if ( inlineSource.empty() )
        {
            OE_WARN << LC << "Inline source for \"" << filename << "\" is empty, and no external file could be found.\n";
        }
    }
    else
    {
        std::string externalSource = URI(path, context).getString(dbOptions);
        if (!externalSource.empty())
        {
            OE_DEBUG << LC << "Loaded external shader " << filename << " from " << path << "\n";
            output = externalSource;
        }
        else
        {
            output = inlineSource;
            useInlineSource = true;
        }
    }

    // replace common tokens:
    osgEarth::replaceIn(output, "$GLSL_VERSION_STR", GLSL_VERSION_STR);
    osgEarth::replaceIn(output, "$GLSL_DEFAULT_PRECISION_FLOAT", GLSL_DEFAULT_PRECISION_FLOAT);

    // If we're using inline source, we have to post-process the string.
    if ( useInlineSource )
    {
        // Replace tokens inserted in the CMakeModules/ConfigureShaders.cmake.in script.
        osgEarth::replaceIn(output, "%EOL%",   "\n");
        osgEarth::replaceIn(output, "%QUOTE%", "\"");
    }

    // Process any "#pragma include" statements
    while(true)
    {
        const std::string token("#pragma include");
        std::string::size_type statementPos = output.find(token);
        if ( statementPos == std::string::npos )
            break;

        std::string::size_type startPos = output.find_first_not_of(" \t", statementPos+token.length());
        if ( startPos == std::string::npos )
            break;

        std::string::size_type endPos = output.find('\n', startPos);
        if ( endPos == std::string::npos )
            break;

        std::string statement( output.substr(statementPos, endPos-statementPos) );
        std::string fileToInclude( trim(output.substr(startPos, endPos-startPos)) );

        // load the source of the included file, and append a newline so we
        // don't break the MULTILINE macro if the last line of the include
        // file is a comment.
        std::string fileSource = Stringify()
            << load(fileToInclude, package, dbOptions)
            << "\n";

        osgEarth::replaceIn(output, statement, fileSource);
    }

    // Process any "#pragma define" statements
    while(true)
    {
        const std::string token("#pragma vp_define");
        std::string::size_type statementPos = output.find(token);
        if ( statementPos == std::string::npos )
            break;

        std::string::size_type startPos = output.find_first_not_of(" \t", statementPos+token.length());
        if ( startPos == std::string::npos )
            break;

        std::string::size_type endPos = output.find('\n', startPos);
        if ( endPos == std::string::npos )
            break;

        std::string statement( output.substr(statementPos, endPos-statementPos) );
        std::string varName( trim(output.substr(startPos, endPos-startPos)) );

        ShaderPackage::DefineMap::const_iterator d = package._defines.find( varName );

        bool defineIt =
            d != package._defines.end() &&
            d->second == true;

        std::string newStatement = Stringify()
            << (defineIt? "#define " : "#undef ")
            << varName;

        osgEarth::replaceIn( output, statement, newStatement );
    }

    // Finally, process any replacements.
    for(ShaderPackage::ReplaceMap::const_iterator i = package._replaces.begin();
        i != package._replaces.end();
        ++i)
    {
        osgEarth::replaceIn( output, i->first, i->second );
    }

    return output;
}

std::string
ShaderLoader::load(const std::string&    filename,
                   const std::string&    inlineSource,
                   const osgDB::Options* dbOptions )
{
    std::string output;
    bool useInlineSource = false;

    URIContext context( dbOptions );
    URI uri(filename, context );

    std::string path = osgDB::findDataFile(filename, dbOptions);
    if ( path.empty() )
    {
        output = inlineSource;
        useInlineSource = true;
    }
    else
    {
        std::string externalSource = URI(path, context).getString(dbOptions);
        if (!externalSource.empty())
        {
            OE_DEBUG << LC << "Loaded external shader " << filename << " from " << path << "\n";
            output = externalSource;
        }
        else
        {
            output = inlineSource;
            useInlineSource = true;
        }
    }

    // replace common tokens:
    osgEarth::replaceIn(output, "$GLSL_VERSION_STR", GLSL_VERSION_STR);
    osgEarth::replaceIn(output, "$GLSL_DEFAULT_PRECISION_FLOAT", GLSL_DEFAULT_PRECISION_FLOAT);

    // If we're using inline source, we have to post-process the string.
    if ( useInlineSource )
    {
        // Replace tokens inserted in the CMakeModules/ConfigureShaders.cmake.in script.
        osgEarth::replaceIn(output, "%EOL%",   "\n");
        osgEarth::replaceIn(output, "%QUOTE%", "\"");
    }

    return output;
}

bool
ShaderLoader::load(VirtualProgram*       vp,
                   const std::string&    filename,
                   const ShaderPackage&  package,
                   const osgDB::Options* dbOptions)
{
    if ( !vp )
    {
        OE_WARN << LC << "Illegal: null VirtualProgram\n";
        return false;
    }

    std::string source = load(filename, package, dbOptions);
    if ( source.empty() )
    {
        OE_WARN << LC << "Failed to load shader source from \"" << filename << "\"\n";
        return false;
    }

    // Remove the quotation marks from the source since they are illegal in GLSL
    replaceIn( source, "\"", " ");

    std::string loc = getPragmaValue(source, "vp_location");
    ShaderComp::FunctionLocation location;
    bool locationSet = true;

    if      ( ciEquals(loc, "vertex_model") )
        location = ShaderComp::LOCATION_VERTEX_MODEL;
    else if ( ciEquals(loc, "vertex_view") )
        location = ShaderComp::LOCATION_VERTEX_VIEW;
    else if ( ciEquals(loc, "vertex_clip") )
        location = ShaderComp::LOCATION_VERTEX_CLIP;
    else if ( ciEquals(loc, "tess_control") || ciEquals(loc, "tessellation_control") )
        location = ShaderComp::LOCATION_TESS_CONTROL;
    else if ( ciEquals(loc, "tess_eval") || ciEquals(loc, "tessellation_eval") || ciEquals(loc, "tessellation_evaluation") || ciEquals(loc, "tess_evaluation") )
        location = ShaderComp::LOCATION_TESS_EVALUATION;
    else if ( ciEquals(loc, "vertex_geometry") || ciEquals(loc, "geometry") )
        location = ShaderComp::LOCATION_GEOMETRY;
    else if ( ciEquals(loc, "fragment" ) )
        location = ShaderComp::LOCATION_FRAGMENT_COLORING;
    else if ( ciEquals(loc, "fragment_coloring") )
        location = ShaderComp::LOCATION_FRAGMENT_COLORING;
    else if ( ciEquals(loc, "fragment_lighting") )
        location = ShaderComp::LOCATION_FRAGMENT_LIGHTING;
    else if ( ciEquals(loc, "fragment_output") )
        location = ShaderComp::LOCATION_FRAGMENT_OUTPUT;
    else
    {
        locationSet = false;
    }

    // If entry point is set, this is a function; otherwise a simple library.
    std::string entryPoint = getPragmaValue(source, "vp_entryPoint");

    // order is optional.
    std::string orderStr = getPragmaValue(source, "vp_order");

    if ( !entryPoint.empty() )
    {
        if ( !locationSet )
        {
            OE_WARN << LC << "Illegal: shader \"" << filename << "\" has invalid #pragma vp_location when vp_entryPoint is set\n";
            return false;
        }

        float order;
        if ( ciEquals(orderStr, "FLT_MAX") || ciEquals(orderStr, "last") )
            order = FLT_MAX;
        else if ( ciEquals(orderStr, "-FLT_MAX") || ciEquals(orderStr, "first") )
            order = -FLT_MAX;
        else
            order = as<float>(orderStr, 1.0f);

        // set the function!
        vp->setFunction( entryPoint, source, location, 0L, order );
    }

    else
    {
        // install as a simple shader.
        if ( locationSet )
        {
            // If a location is set, install in that location only
            osg::Shader::Type type =
                location == ShaderComp::LOCATION_VERTEX_MODEL || location == ShaderComp::LOCATION_VERTEX_VIEW || location == ShaderComp::LOCATION_VERTEX_CLIP ? osg::Shader::VERTEX :
                osg::Shader::FRAGMENT;

            osg::Shader* shader = new osg::Shader(type, source);
            shader->setName( filename );
            vp->setShader( filename, shader );
        }

        else
        {
            // If no location was set, install in all stages.
            osg::Shader::Type types[5] = { osg::Shader::VERTEX, osg::Shader::FRAGMENT, osg::Shader::GEOMETRY, osg::Shader::TESSCONTROL, osg::Shader::TESSEVALUATION };
            for(int i=0; i<5; ++i)
            {
                osg::Shader* shader = new osg::Shader(types[i], source);
                std::string name = Stringify() << filename + "_" + shader->getTypename();
                shader->setName( name );
                vp->setShader( name, shader );
            }
        }
    }

    return true;
}

bool
ShaderLoader::unload(VirtualProgram*       vp,
                     const std::string&    filename,
                     const ShaderPackage&  package,
                     const osgDB::Options* dbOptions)
{
    if ( !vp )
    {
        // fail quietly
        return false;
    }

    std::string source = load(filename, package, dbOptions);
    if ( source.empty() )
    {
        OE_WARN << LC << "Failed to unload shader source from \"" << filename << "\"\n";
        return false;
    }

    std::string entryPoint = getPragmaValue(source, "vp_entryPoint");
    if ( !entryPoint.empty() )
    {
        vp->removeShader( entryPoint );
    }
    else
    {
        vp->removeShader( filename );
    }
    return true;
}

//...................................................................

void
ShaderPackage::define(const std::string& name,
                      bool               defOrUndef)
{
    _defines[name] = defOrUndef;
}

void
ShaderPackage::replace(const std::string& pattern,
                       const std::string& value)
{
    _replaces[pattern] = value;
}

bool
ShaderPackage::load(VirtualProgram*       vp,
                    const std::string&    filename,
                    const osgDB::Options* dbOptions) const
{
    return ShaderLoader::load(vp, filename, *this, dbOptions);
}

bool
ShaderPackage::unload(VirtualProgram*       vp,
                      const std::string&    filename,
                      const osgDB::Options* dbOptions) const
{
    return ShaderLoader::unload(vp, filename, *this, dbOptions);
}

bool
ShaderPackage::loadAll(VirtualProgram*       vp,
                       const osgDB::Options* dbOptions) const
{
    int oks = 0;
    for(SourceMap::const_iterator i = _sources.begin(); i != _sources.end(); ++i)
    {
        oks += load( vp, i->first ) ? 1 : 0;
    }
    return oks == _sources.size();
}

bool
ShaderPackage::unloadAll(VirtualProgram*       vp,
                          const osgDB::Options* dbOptions) const
{
    int oks = 0;
    for(SourceMap::const_iterator i = _sources.begin(); i != _sources.end(); ++i)
    {
        oks += unload( vp, i->first ) ? 1 : 0;
    }
    return oks == _sources.size();
}
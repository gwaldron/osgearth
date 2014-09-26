/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthUtil/ChromaKeyColorFilter>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osg/Program>
#include <OpenThreads/Atomic>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    static OpenThreads::Atomic s_uniformNameGen;

    static const char* s_localShaderSource =

        "#version 110\n"

        "uniform vec3  __COLOR_UNIFORM_NAME__;\n"
        "uniform float __DISTANCE_UNIFORM_NAME__;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{ \n"
        "    float dist = distance(color.rgb, __COLOR_UNIFORM_NAME__); \n"
        "    if (dist <= __DISTANCE_UNIFORM_NAME__) color.a = 0.0;\n"
        // feathering:
        //"    if (dist <= __DISTANCE_UNIFORM_NAME__) color.a = (dist/__DISTANCE_UNIFORM_NAME__); \n" 
        "} \n";
}


//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_chromakeyColorFilter_"
#define COLOR_UNIFORM_PREFIX  "osgearthutil_u_chromakey_color_"
#define DISTANCE_UNIFORM_PREFIX  "osgearthutil_u_chromakey_distance_"

//---------------------------------------------------------------------------

ChromaKeyColorFilter::ChromaKeyColorFilter(void)
{
    init();
}

void ChromaKeyColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    _instanceId = (++s_uniformNameGen) - 1;
    _color = new osg::Uniform(osg::Uniform::FLOAT_VEC3, (osgEarth::Stringify() << COLOR_UNIFORM_PREFIX << _instanceId));    
    //Default to black
    _color->set( osg::Vec3(0.0f, 0.0f, 0.0f) );
    _distance = new osg::Uniform(osg::Uniform::FLOAT, (osgEarth::Stringify() << DISTANCE_UNIFORM_PREFIX << _instanceId));    
    _distance->set( 0.0f );
}

void ChromaKeyColorFilter::setColor(const osg::Vec3f& color)
{
    _color->set( color );
}

osg::Vec3f ChromaKeyColorFilter::getColor() const
{
    osg::Vec3f value;
    _color->get( value );
    return value;
}

void ChromaKeyColorFilter::setDistance(float distance)
{
    _distance->set( distance );
}

float ChromaKeyColorFilter::getDistance() const
{
    float value;
    _distance->get( value );
    return value;
}



std::string ChromaKeyColorFilter::getEntryPointFunctionName(void) const
{
    return (osgEarth::Stringify() << FUNCTION_PREFIX << _instanceId);
}

void ChromaKeyColorFilter::install(osg::StateSet* stateSet) const
{
    // safe: will not add twice.
    stateSet->addUniform(_color.get());
    stateSet->addUniform(_distance.get());

    osgEarth::VirtualProgram* vp = dynamic_cast<osgEarth::VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if (vp)
    {
        // build the local shader (unique per instance). We will
        // use a template with search and replace for this one.
        std::string entryPoint = osgEarth::Stringify() << FUNCTION_PREFIX << _instanceId;
        std::string code = s_localShaderSource;
        osgEarth::replaceIn(code, "__COLOR_UNIFORM_NAME__", _color->getName());
        osgEarth::replaceIn(code, "__DISTANCE_UNIFORM_NAME__", _distance->getName());
        osgEarth::replaceIn(code, "__ENTRY_POINT__", entryPoint);

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        vp->setShader(entryPoint, main);
    }
}



//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( chroma_key, osgEarth::Util::ChromaKeyColorFilter );


ChromaKeyColorFilter::ChromaKeyColorFilter(const Config& conf)
{
    init();

    osg::Vec3f val;
    val[0] = conf.value("r", 0.0);
    val[1] = conf.value("g", 0.0);
    val[2] = conf.value("b", 0.0);
    setColor( val );

    float distance = 0.0f;
    distance = conf.value("distance", 0.0f);
    setDistance( distance );
}

Config
ChromaKeyColorFilter::getConfig() const
{
    osg::Vec3f val = getColor();
    Config conf("chroma_key");
    conf.add( "r", val[0] );
    conf.add( "g", val[1] );
    conf.add( "b", val[2] );

    if ( getDistance() != 0.0f )
        conf.add( "distance", getDistance() );

    return conf;
}

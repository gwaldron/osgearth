/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/NormalMap>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[NormalMap] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char* vs =
        "varying vec3 light; \n"
        "varying vec3 view; \n"

        "void oe_normalmap_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    vec3 tangent = normalize(cross(gl_Normal, vec3(0,-1,0))); \n"

        "    vec3 n = normalize(gl_NormalMatrix * gl_Normal); \n"
        "    vec3 t = normalize(gl_NormalMatrix * tangent); \n"
        "    vec3 b = cross(n, t); \n"

        "    vec3 tmp = gl_LightSource[0].position.xyz; \n"
        "    light.x = dot(tmp, t); \n"
        "    light.y = dot(tmp, b); \n"
        "    light.z = dot(tmp, n); \n"

        "    tmp = -VertexVIEW.xyz; \n"
        "    view.x = dot(tmp, t); \n"
        "    view.y = dot(tmp, b); \n"
        "    view.z = dot(tmp, n); \n"
        "} \n";

    const char* fs =
        "uniform sampler2D oe_normalmap_tex; \n"
        "uniform float oe_normalmap_intensity; \n"
        "varying vec4 oe_layer_tilec; \n"
        "varying vec3 light; \n"
        "varying vec3 view; \n"

        "void oe_normalmap_fragment(inout vec4 color) \n"
        "{\n"
        "    vec3  L = normalize(light); \n"
        "    vec3  V = normalize(view); \n"
        "    vec3  N = normalize( texture2D(oe_normalmap_tex, oe_layer_tilec.st).xyz * 2.0 - 1.0 ); \n"
        "    float D = max(dot(L, N), 0.0); \n"
        //"    float S = pow(clamp(dot(reflect(-L,N),V),0.0,1.0), gl_FrontMaterial.shininess); \n"
        "    vec4  diffuse  = gl_FrontLightProduct[0].diffuse * D; \n"
        "    vec4  ambient  = gl_FrontLightProduct[0].ambient; \n"
        //"    vec4  specular = gl_FrontLightProduct[0].specular * S; \n"
        //"    color.rgb *= clamp(diffuse.rgb + ambient.rgb + specular.rgb, 0.0, 1.0); \n"
        "    color.rgb *= clamp(diffuse.rgb + ambient.rgb, 0.0, 1.0); \n"
        "}\n";
}


NormalMap::NormalMap() :
TerrainEffect(),
_intensity( 1.0f )
{
    init();
}


void
NormalMap::init()
{
    _intensityUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_normalmap_intensity");
    _intensityUniform->set( _intensity.get() );
}


NormalMap::~NormalMap()
{
    //nop
}


void
NormalMap::setIntensity(float i)
{
    if ( i != _intensity.get() )
    {
        _intensity = i;
        _intensityUniform->set( i );
    }
}

void
NormalMap::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();
        if ( _layer.valid() )
        {
            OE_NOTICE << LC << "Installing layer " << _layer->getName() << " as normal map" << std::endl;
            int unit = *_layer->shareImageUnit();
            stateset->getOrCreateUniform("oe_normalmap_tex", osg::Uniform::SAMPLER_2D)->set(unit);
        }
        
        stateset->addUniform( _intensityUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

        vp->setFunction( "oe_normalmap_vertex",   vs, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_normalmap_fragment", fs, ShaderComp::LOCATION_FRAGMENT_LIGHTING );
    }
}


void
NormalMap::onUninstall(TerrainEngineNode* engine)
{
    OE_WARN << LC << "Uninstall NYI." << std::endl;
}



NormalMap::NormalMap(const Config& conf, Map* map) :
TerrainEffect(),
_intensity   (1.0)
{
    mergeConfig(conf);

    if ( map && _layerName.isSet() )
    {
        setNormalMapLayer( map->getImageLayerByName(*_layerName) );
    }

    init();
}

void
NormalMap::mergeConfig(const Config& conf)
{
    conf.getIfSet( "layer",     _layerName );
    conf.getIfSet( "intensity", _intensity );
}

Config
NormalMap::getConfig() const
{
    optional<std::string> layername;

    if ( _layer.valid() && !_layer->getName().empty() )
        layername = _layer->getName();

    Config conf("normal_map");
    conf.addIfSet( "layer",     layername );
    conf.addIfSet( "intentisy", _intensity );
    return conf;
}

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
#include <osgEarthUtil/DetailTexture>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[DetailTexture] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char* vs =
        "uniform vec4  oe_tile_key; \n"
        "varying vec4  oe_layer_tilec; \n"
        "uniform float oe_dtex_L0; \n"
        "varying vec2  oe_dtex_tc; \n"

        "int oe_dtex_ipow(in int x, in int y) { \n"
        "   int r = 1; \n"
        "   while( y > 0 ) { \n"
        "       r *= x; \n"
        "       --y; \n"
        "   } \n"
        "   return r; \n"
        "}\n"

        "void oe_dtex_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float dL = oe_tile_key.z - oe_dtex_L0; \n"
        "    float twoPowDeltaL = float(oe_dtex_ipow(2, int(abs(dL)))); \n"
        "    float factor = dL >= 0.0 ? twoPowDeltaL : 1.0/twoPowDeltaL; \n"

        "    vec2 a = floor(oe_tile_key.xy / factor); \n"
        "    vec2 b = a * factor; \n"
        "    vec2 c = (a+1.0) * factor; \n"
        "    vec2 offset = (oe_tile_key.xy-b)/(c-b); \n"
        "    vec2 scale = vec2(1.0/factor); \n"

        "    oe_dtex_tc = (oe_layer_tilec.st * scale) + offset; \n"
        "} \n";

    const char* fs =
        "uniform vec4      oe_tile_key; \n"
        "uniform float     oe_dtex_L0; \n"
        "uniform sampler2D oe_dtex_tex; \n"
        "uniform float     oe_dtex_intensity; \n"
        "varying vec2      oe_dtex_tc; \n"

        "void oe_dtex_fragment(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_tile_key.z >= oe_dtex_L0 ) \n"
        "    { \n"
        "        vec4 texel = texture2D(oe_dtex_tex, oe_dtex_tc); \n"
        "        if ( oe_tile_key.z >= oe_dtex_L0+3.0 ) \n"
        "        { \n"
        "            texel += texture2D(oe_dtex_tex, oe_dtex_tc*8.0)-0.5; \n"
        "            if ( oe_tile_key.z >= oe_dtex_L0+6.0 ) \n"
        "            { \n"
        "                texel += texture2D(oe_dtex_tex, oe_dtex_tc*32.0)-0.5; \n"
        "                if ( oe_tile_key.z >= oe_dtex_L0+9.0 ) \n"
        "                { \n"
        "                    texel += texture2D(oe_dtex_tex, oe_dtex_tc*64.0)-0.5; \n"
        "                } \n"
        "            } \n"
        "        } \n"
        "        texel.rgb -= 0.5; \n"
        "        color.rgb = clamp( color.rgb + (texel.rgb*oe_dtex_intensity), 0.0, 1.0 ); \n"
        "    } \n"
        "} \n";
}


DetailTexture::DetailTexture() :
TerrainEffect(),
_unit        ( 1 ),
_baseLOD     ( 10 ),
_intensity   ( 0.25f )
{
    _samplerUniform   = new osg::Uniform(osg::Uniform::SAMPLER_2D, "oe_dtex_tex");
    _samplerUniform->set( (int)_unit );

    _baseLODUniform   = new osg::Uniform(osg::Uniform::FLOAT, "oe_dtex_L0");
    _baseLODUniform->set( (float)_baseLOD );

    _intensityUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_dtex_intensity");
    _intensityUniform->set( _intensity );

    _texture = new osg::Texture2D();
    _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
    _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _texture->setResizeNonPowerOfTwoHint( false );
}


DetailTexture::~DetailTexture()
{
    //nop
}


void
DetailTexture::setImageUnit(unsigned unit)
{
    _unit = std::min( (int)unit, Registry::capabilities().getMaxGPUTextureUnits()-1 );
    _samplerUniform->set( _unit );
}


void
DetailTexture::setBaseLOD(unsigned lod)
{
    _baseLOD = lod;
    _baseLODUniform->set( (float)_baseLOD );
}


void
DetailTexture::setIntensity(float intensity)
{
    _intensity = osg::clampBetween( intensity, 0.0f, 1.0f );
    _intensityUniform->set( _intensity );
}


void
DetailTexture::setImage(const osg::Image* image)
{
    if ( image )
        _texture->setImage( const_cast<osg::Image*>(image) );
}


void
DetailTexture::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        stateset->addUniform( _baseLODUniform.get() );
        stateset->addUniform( _samplerUniform.get() );
        stateset->addUniform( _intensityUniform.get() );

        stateset->setTextureAttributeAndModes( _unit, _texture.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_dtex_vertex",   vs, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "oe_dtex_fragment", fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }
}


void
DetailTexture::onUninstall(TerrainEngineNode* engine)
{
    OE_WARN << LC << "Uninstall NYI" << std::endl;
}

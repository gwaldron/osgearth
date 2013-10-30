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
#include <osgEarthUtil/LODBlending>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[LODBlending] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    // This shader will morph elevation from old heights to new heights as
    // installed in the terrain tile's vertex attributes. oe_terrain_attr[3] holds
    // the new value; oe_terrain_attr[3] holds the old one. 
    //
    // We use two methods: distance to vertex and time. The morph ratio is
    // a function of the distance from the camera to the vertex (taking into
    // consideration the tile range factor), but when we limit that based on
    // a timer. This prevents fast zooming from skipping the morph altogether.
    //
    // It will also transition between a parent texture and the current texture.
    //
    // Caveats: You can still fake out the morph by zooming around very quickly.
    // Also, it will only morph properly if you use odd-numbers post spacings
    // in your terrain tile. (See MapOptions::elevation_tile_size). Finally,
    // a large PAGEDLOD cache will negate the blending effect when zooming out
    // and then back in. See MPGeometry.

    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "attribute vec4 oe_terrain_attr; \n"
        "attribute vec4 oe_terrain_attr2; \n"
        "varying vec3 oe_Normal; \n"

        "uniform float oe_min_tile_range_factor; \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float osg_FrameTime; \n"
        "uniform float oe_tile_birthtime; \n"
        "uniform float oe_lodblend_delay; \n"
        "uniform float oe_lodblend_duration; \n"
        "uniform float oe_lodblend_vscale; \n"

        "uniform mat4 oe_layer_parent_matrix; \n"
        "varying vec4 oe_layer_texc; \n"
        "varying vec4 oe_lodblend_texc; \n"
        "varying float oe_lodblend_r; \n"

        "void oe_lodblend_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float radius     = oe_tile_key.w; \n"
        "    float near       = oe_min_tile_range_factor*radius; \n"
        "    float far        = near + radius*2.0; \n"
        "    vec4  VertexVIEW = gl_ModelViewMatrix * VertexMODEL; \n"
        "    float d          = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    float r_dist     = clamp((d-near)/(far-near), 0.0, 1.0); \n"

        "    float r_time     = 1.0 - clamp(osg_FrameTime-(oe_tile_birthtime+oe_lodblend_delay), 0.0, oe_lodblend_duration)/oe_lodblend_duration; \n"
        "    float r          = max(r_dist, r_time); \n"

        "    vec3  upVector   = oe_terrain_attr.xyz; \n"
        "    float elev       = oe_terrain_attr.w; \n"
        "    float elevOld    = oe_terrain_attr2.w; \n"

        "    vec3  vscaleOffset = upVector * elev * (oe_lodblend_vscale-1.0); \n"
        "    vec3  blendOffset  = upVector * r * oe_lodblend_vscale * (elevOld-elev); \n"
        "    VertexMODEL       += vec4( (vscaleOffset + blendOffset)*VertexMODEL.w, 0.0 ); \n"

        "    oe_lodblend_texc    = oe_layer_parent_matrix * oe_layer_texc; \n"
        "    oe_lodblend_r       = oe_layer_parent_matrix[0][0] > 0.0 ? r : 0.0; \n" // obe?

        "    oe_Normal = normalize(mix(normalize(oe_Normal), oe_terrain_attr2.xyz, r)); \n"
        "} \n";

    const char* fs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform vec4 oe_tile_key; \n"
        "uniform int oe_layer_uid; \n"
        "varying vec4 oe_lodblend_texc; \n"
        "varying float oe_lodblend_r; \n"
        "uniform sampler2D oe_layer_tex_parent; \n"

        "void oe_lodblend_fragment(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_layer_uid >= 0 ) \n"
        "    { \n"
        "        vec4 texel = texture2D(oe_layer_tex_parent, oe_lodblend_texc.st); \n"
        "        float enable = step(0.0001, texel.a); \n"          // did we get a parent texel?
        "        texel.rgb = mix(color.rgb, texel.rgb, enable); \n" // if not, use the incoming color for the blend
        "        texel.a = mix(0.0, color.a, enable); \n"           // ...and blend from alpha=0 for a fade-in effect.
        "        color = mix(color, texel, oe_lodblend_r); \n"
        "    } \n"
        "} \n";
}


LODBlending::LODBlending() :
TerrainEffect(),
_delay       ( 0.0f ),
_duration    ( 0.25f ),
_vscale      ( 1.0f )
{
    init();
}


LODBlending::LODBlending(const Config& conf) :
TerrainEffect(),
_delay       ( 0.0f ),
_duration    ( 0.25f ),
_vscale      ( 1.0f )
{
    mergeConfig(conf);
    init();
}


void
LODBlending::init()
{
    _delayUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_delay");
    _delayUniform->set( (float)*_delay );

    _durationUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_duration");
    _durationUniform->set( (float)*_duration );

    _vscaleUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_vscale");
    _vscaleUniform->set( (float)*_vscale );
}


void
LODBlending::setDelay(float delay)
{
    if ( delay != _delay.get() )
    {
        _delay = osg::clampAbove( delay, 0.0f );
        _delayUniform->set( _delay.get() );
    }
}


void
LODBlending::setDuration(float duration)
{
    if ( duration != _duration.get() )
    {
        _duration = osg::clampAbove( duration, 0.0f );
        _durationUniform->set( _duration.get() );
    }
}


void
LODBlending::setVerticalScale(float vscale)
{
    if ( vscale != _vscale.get() )
    {
        _vscale = osg::clampAbove( vscale, 0.0f );
        _vscaleUniform->set( _vscale.get() );
    }
}


void
LODBlending::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        stateset->addUniform( _delayUniform.get() );
        stateset->addUniform( _durationUniform.get() );
        stateset->addUniform( _vscaleUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName( "osgEarth::Util::LODBlending" );
        vp->setFunction( "oe_lodblend_vertex",   vs, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "oe_lodblend_fragment", fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }
}


void
LODBlending::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( _delayUniform.get() );
            stateset->removeUniform( _durationUniform.get() );
            stateset->removeUniform( _vscaleUniform.get() );

            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                vp->removeShader( "oe_lodblend_vertex" );
                vp->removeShader( "oe_lodblend_fragment" );
            }
        }
    }
}


//-------------------------------------------------------------


void
LODBlending::mergeConfig(const Config& conf)
{
    conf.getIfSet( "delay",    _delay );
    conf.getIfSet( "duration", _duration );
    conf.getIfSet( "vertical_scale", _vscale );
}

Config
LODBlending::getConfig() const
{
    Config conf("lod_blending");
    conf.addIfSet( "delay",    _delay );
    conf.addIfSet( "duration", _duration );
    conf.addIfSet( "vertical_scale", _vscale );
    return conf;
}

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
#include <osgEarthUtil/ElevationMorph>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[ElevationMorph] "

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
    // Caveats: You can still fake out the morph by zooming around very quickly.
    // Also, it will only morph properly if you use odd-numbers post spacings
    // in your terrain tile. (See MapOptions::elevation_tile_size).

    const char* vs =
        "attribute vec4 oe_terrain_attr; \n"
        "attribute vec4 oe_terrain_attr2; \n"
        "uniform float oe_min_tile_range_factor; \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float osg_FrameTime; \n"
        "uniform float oe_tile_birthtime; \n"
        "uniform float oe_morph_delay; \n"
        "uniform float oe_morph_duration; \n"

        "uniform mat4 oe_layer_parent_matrix; \n"
        "varying vec4 oe_layer_texc; \n"
        "varying vec4 oe_morph_texc; \n"
        "varying float oe_morph_r; \n"

        "void oe_morph_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float radius     = oe_tile_key.w; \n"
        "    float near       = oe_min_tile_range_factor*radius; \n"
        "    float far        = near + radius*2.0; \n"
        "    vec4  VertexVIEW = gl_ModelViewMatrix * VertexMODEL; \n"
        "    float d          = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    float r_dist     = clamp((d-near)/(far-near), 0.0, 1.0); \n"

        "    float r_time     = 1.0 - clamp(osg_FrameTime-(oe_tile_birthtime+oe_morph_delay), 0.0, oe_morph_duration)/oe_morph_duration; \n"
        "    float r          = max(r_dist, r_time); \n"

        "    vec3  upVector   = oe_terrain_attr.xyz; \n"
        "    float elev       = oe_terrain_attr.w; \n"
        "    float elevOld    = oe_terrain_attr2.w; \n"
        "    vec3  offset     = upVector * r * (elevOld - elev); \n"
        "    VertexMODEL      = VertexMODEL + vec4(offset/VertexMODEL.w, 0.0); \n"
        
        "    oe_morph_texc    = oe_layer_parent_matrix * oe_layer_texc; \n"
        "    oe_morph_r       = oe_layer_parent_matrix[0][0] > 0.0 ? r : 0.0; \n"
        "} \n";

    const char* fs =
        "uniform vec4 oe_tile_key; \n"
        "uniform float oe_layer_opacity; \n"
        "varying vec4 oe_morph_texc; \n"
        "varying float oe_morph_r; \n"
        "uniform sampler2D oe_layer_tex_parent; \n"

        "void oe_morph_fragment(inout vec4 color) \n"
        "{ \n"
        "    vec4 texelpma = texture2D(oe_layer_tex_parent, oe_morph_texc.st) * oe_layer_opacity; \n"
        "    color = mix(color, texelpma, oe_morph_r); \n"
        "} \n";
}


ElevationMorph::ElevationMorph() :
TerrainEffect(),
_delay       ( 0.0f ),
_duration    ( 0.25f )
{
    init();
}


ElevationMorph::ElevationMorph(const Config& conf) :
TerrainEffect(),
_delay       ( 0.0f ),
_duration    ( 0.25f )
{
    mergeConfig(conf);
    init();
}


void
ElevationMorph::init()
{
    _delayUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_morph_delay");
    _delayUniform->set( (float)*_delay );

    _durationUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_morph_duration");
    _durationUniform->set( (float)*_duration );
}


void
ElevationMorph::setDelay(float delay)
{
    if ( delay != _delay.get() )
    {
        _delay = osg::clampAbove( delay, 0.0f );
        _delayUniform->set( _delay.get() );
    }
}


void
ElevationMorph::setDuration(float duration)
{
    if ( duration != _duration.get() )
    {
        _duration = osg::clampAbove( duration, 0.0f );
        _durationUniform->set( _duration.get() );
    }
}


void
ElevationMorph::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        stateset->addUniform( _delayUniform.get() );
        stateset->addUniform( _durationUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_morph_vertex",   vs, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "oe_morph_fragment", fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }
}


void
ElevationMorph::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( _delayUniform.get() );
            stateset->removeUniform( _durationUniform.get() );

            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                vp->removeShader( "oe_morph_vertex" );
                vp->removeShader( "oe_morph_fragment" );
            }
        }
    }
}


//-------------------------------------------------------------


void
ElevationMorph::mergeConfig(const Config& conf)
{
    conf.getIfSet( "delay",    _delay );
    conf.getIfSet( "duration", _duration );
}

Config
ElevationMorph::getConfig() const
{
    Config conf("elevation_morph");
    conf.addIfSet( "delay",    _delay );
    conf.addIfSet( "duration", _duration );
    return conf;
}

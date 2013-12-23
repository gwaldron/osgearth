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
#include <osgEarth/ImageUtils>

#define LC "[DetailTexture] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform vec4 oe_tile_key; \n"
        "varying vec4 oe_layer_tilec; \n"
        "uniform float oe_detail_L0; \n"
        "varying vec2 oe_detail_tc; \n"
        "uniform float oe_detail_scale; \n"

        "int oe_detail_ipow(in int x, in int y) { \n"
        "   int r = 1; \n"
        "   while( y > 0 ) { \n"
        "       r *= x; \n"
        "       --y; \n"
        "   } \n"
        "   return r; \n"
        "}\n"

        "void oe_detail_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float dL = oe_tile_key.z - oe_detail_L0; \n"
        "    float twoPowDeltaL = float(oe_detail_ipow(2, int(abs(dL)))); \n"
        "    float factor = dL >= 0.0 ? twoPowDeltaL : 1.0/twoPowDeltaL; \n"

        "    vec2 a = floor(oe_tile_key.xy / factor); \n"
        "    vec2 b = a * factor; \n"
        "    vec2 c = (a+1.0) * factor; \n"
        "    vec2 offset = (oe_tile_key.xy-b)/(c-b); \n"
        "    vec2 scale = vec2(1.0/factor); \n"

        "    float tscale = pow(2.0, oe_detail_scale); \n"
        "    oe_detail_tc = tscale * (oe_layer_tilec.st * scale) + offset; \n"
        "} \n";

    const char* fs =
        "#version " GLSL_VERSION_STR "\n"
        "#extension GL_EXT_texture_array : enable\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform vec4 oe_tile_key; \n"
        "uniform float oe_detail_L0; \n"
        "uniform sampler2D oe_detail_mask; \n"
        "uniform sampler2DArray oe_detail_tex; \n"
        "uniform float oe_detail_intensity; \n"
        "varying vec2 oe_detail_tc; \n"
        "varying vec4 oe_layer_tilec; \n"

        "void oe_detail_fragment(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_tile_key.z >= oe_detail_L0 ) \n"
        "    { \n"
        "        vec4 t0 = texture2DArray(oe_detail_tex, vec3(oe_detail_tc,0)); \n"
        "        vec4 t1 = texture2DArray(oe_detail_tex, vec3(oe_detail_tc,1)); \n"
        "        vec4 t2 = texture2DArray(oe_detail_tex, vec3(oe_detail_tc,2)); \n"
        "        vec4 t3 = texture2DArray(oe_detail_tex, vec3(oe_detail_tc,3)); \n"

        "        vec4 m = texture2D(oe_detail_mask, oe_layer_tilec.st); \n"

        "        vec4 detail = t0*m[0] + t1*m[1] + t2*m[2] + t3*m[3];\n"
        //"        vec3 luminosity = vec3(detail.r*0.2125 + detail.g*0.7154 + detail.b*0.0721);\n"
        "        color = mix(color, detail, oe_detail_intensity);\n"
        "    } \n"
        "} \n";
}


DetailTexture::DetailTexture() :
TerrainEffect(),
_startLOD    ( 10 ),
_intensity   ( 1.0f ),
_scale       ( 1.0f )
{
    init();
}

DetailTexture::DetailTexture(const Config& conf, const Map* map) :
TerrainEffect(),
_startLOD    ( 10 ),
_intensity   ( 1.0f ),
_scale       ( 1.0f )
{
    mergeConfig(conf);

    if ( map && _maskLayerName.isSet() )
    {
        _maskLayer = map->getImageLayerByName(*_maskLayerName);
    }

    init();
}


void
DetailTexture::init()
{
    // negative means unset:
    _unit = -1;

    _startLODUniform   = new osg::Uniform(osg::Uniform::FLOAT, "oe_detail_L0");
    _startLODUniform->set( (float)_startLOD.get() );

    _intensityUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_detail_intensity");
    _intensityUniform->set( _intensity.get() );

    _scaleUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_detail_scale");
    _scaleUniform->set( _scale.get() );


    _texture = new osg::Texture2DArray();
    _texture->setTextureSize(1024, 1024, 4);
    _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
    _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    _texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _texture->setResizeNonPowerOfTwoHint( false );

    static char* textures[] = {
        "E:/data/textures/seamless/dirt5.jpg",
        "E:/data/textures/seamless/rock.jpg",
        "E:/data/textures/seamless/test.jpg", //grass1.jpg",
        "E:/data/textures/seamless/water.jpg"
    };

    for(unsigned i=0; i<4; ++i) {
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile(textures[i]);
        osg::ref_ptr<osg::Image> imageResized;
        ImageUtils::resizeImage( image.get(), 1024, 1024, imageResized );
        _texture->setImage(i, imageResized.get());
    }
}


void
DetailTexture::setStartLOD(unsigned lod)
{
    if ( lod != _startLOD.get() )
    {
        _startLOD = lod;
        _startLODUniform->set( (float)_startLOD.get() );
    }
}


void
DetailTexture::setIntensity(float intensity)
{
    _intensity = osg::clampBetween( intensity, 0.0f, 1.0f );
    _intensityUniform->set( _intensity.get() );
}


void
DetailTexture::setScale(float scale)
{
    _scale = osg::clampAbove( scale, 1.0f );
    _scaleUniform->set( _scale.get() );
}


void
DetailTexture::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_unit) )
        {
            _samplerUniform = stateset->getOrCreateUniform( "oe_detail_tex", osg::Uniform::SAMPLER_2D_ARRAY );
            _samplerUniform->set( _unit );
            stateset->setTextureAttributeAndModes( _unit, _texture.get() );
        }

        if ( _maskLayer.valid() )
        {
            int unit = *_maskLayer->shareImageUnit();
            _maskUniform = stateset->getOrCreateUniform("oe_detail_mask", osg::Uniform::SAMPLER_2D);
            _maskUniform->set(unit);
            OE_NOTICE << LC << "Installed layer " << _maskLayer->getName() << " as texture mask on unit " << unit << std::endl;
        }
        else
        {
            exit(-1);
        }

        stateset->addUniform( _startLODUniform.get() );
        stateset->addUniform( _intensityUniform.get() );
        stateset->addUniform( _scaleUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_detail_vertex",   vs, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "oe_detail_fragment", fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }
}


void
DetailTexture::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        stateset->removeUniform( _startLODUniform.get() );
        stateset->removeUniform( _intensityUniform.get() );
        stateset->removeUniform( _scaleUniform.get() );

        if ( _samplerUniform.valid() )
        {
            int unit;
            _samplerUniform->get(unit);
            stateset->removeUniform( _samplerUniform.get() );
            stateset->removeTextureAttribute(unit, osg::StateAttribute::TEXTURE);
        }

        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_detail_vertex" );
            vp->removeShader( "oe_detail_fragment" );
        }
    }

    if ( _unit >= 0 )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( _unit );
        _unit = -1;
    }
}




//-------------------------------------------------------------

void
DetailTexture::mergeConfig(const Config& conf)
{
    conf.getIfSet( "start_lod",  _startLOD );
    conf.getIfSet( "intensity",  _intensity );
    conf.getIfSet( "scale",      _scale );
    conf.getIfSet( "mask_layer", _maskLayerName );
}

Config
DetailTexture::getConfig() const
{
    optional<std::string> layername;

    if ( _maskLayer.valid() && !_maskLayer->getName().empty() )
        layername = _maskLayer->getName();

    Config conf("detail_texture");
    conf.addIfSet( "start_lod",  _startLOD );
    conf.addIfSet( "intensity",  _intensity );
    conf.addIfSet( "scale",      _scale );
    conf.addIfSet( "mask_layer", layername );

    return conf;
}

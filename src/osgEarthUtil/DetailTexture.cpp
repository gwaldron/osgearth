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
#include <osgEarth/URI>

#define LC "[DetailTexture] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform vec4 oe_tile_key; \n"
        "uniform float oe_detail_L0; \n"
        "uniform float oe_detail_scale; \n"
        "uniform float oe_detail_attenuation_distance; \n"

        "varying vec4 oe_layer_tilec; \n"
        "varying vec2 oe_detail_tc; \n"
        "varying float oe_detail_atten_factor; \n"

        "int oe_detail_ipow(in int x, in int y) { \n"
        "   int r = 1; \n"
        "   while( y > 0 ) { \n"
        "       r *= x; \n"
        "       --y; \n"
        "   } \n"
        "   return r; \n"
        "}\n"

        "void oe_detail_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    float dL = oe_tile_key.z - oe_detail_L0; \n"
        "    float twoPowDeltaL = float(oe_detail_ipow(2, int(abs(dL)))); \n"
        "    float factor = dL >= 0.0 ? twoPowDeltaL : 1.0/twoPowDeltaL; \n"

        "    vec2 a = floor(oe_tile_key.xy / factor); \n"
        "    vec2 b = a * factor; \n"
        "    vec2 c = (a+1.0) * factor; \n"
        "    vec2 offset = (oe_tile_key.xy-b)/(c-b); \n"
        "    vec2 scale = vec2(1.0/factor); \n"

        "    float tscale = pow(2.0, oe_detail_scale-1.0); \n"
        "    oe_detail_tc = tscale * ((oe_layer_tilec.st * scale) + offset); \n"
        
        "    float r = 1.0-((-VertexVIEW.z/VertexVIEW.w)/oe_detail_attenuation_distance);\n"
        "    oe_detail_atten_factor = clamp(r, 0.0, 1.0); \n"
        "} \n";


    std::string generateFragmentShader(unsigned numTextures, unsigned numOctaves)
    {
        std::stringstream buf;

        buf <<
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
            "varying float oe_detail_atten_factor; \n"

            "void oe_detail_fragment(inout vec4 color) \n"
            "{ \n"
            "    if ( oe_tile_key.z >= oe_detail_L0 ) \n"
            "    { \n"
            //"        vec4 m = texture2D(oe_detail_mask, oe_layer_tilec.st); \n"
            "        vec4 m = 0.25 * (\n"
            //"                 texture2D(oe_detail_mask, oe_layer_tilec.st) + \n"
            "                 texture2D(oe_detail_mask, oe_layer_tilec.st + vec2(-0.4,0.0)) + \n"
            "                 texture2D(oe_detail_mask, oe_layer_tilec.st + vec2( 0.4,0.0)) + \n"
            "                 texture2D(oe_detail_mask, oe_layer_tilec.st + vec2( 0.0,-0.4)) + \n"
            "                 texture2D(oe_detail_mask, oe_layer_tilec.st + vec2( 0.0, 0.4))); \n"
            "        vec4 detail = vec4(0.0); \n";

        if ( numOctaves > 1 )
        {
            buf <<
                "        float weight[" << numOctaves << "];\n"
                "        float interval = 1.0/" << numOctaves << ".0;\n"
                "        float d, w;\n"
                "        float total = 0.0;\n"
                    "        float a = oe_detail_atten_factor; \n"
                    "        a = a*a*a*a*a*a*a; \n"; // weights the transitions to be close to the eyepoint

            for(unsigned oct=0; oct<numOctaves; ++oct)
            {
                buf <<
                    "        d = abs( (interval+(interval*" << oct << ".0)) - a);\n"
                    "        w = (1.0-d)*(1.0-d)*(1.0-d); \n"
                    "        total += w*w; \n"
                    "        weight[" << oct << "] = w;\n";
            }
                
            buf <<
                "        float sqrttotal = sqrt(total);\n";

            for(unsigned oct=0; oct<numOctaves; ++oct)
            {
                buf <<
                    "        w = weight["<<oct<<"] / sqrttotal; \n";

                unsigned ofx = 1+(5*oct); // todo: make that octave jump configurable?
                for(unsigned i=0; i<numTextures; ++i)
                {
                    buf <<
                    "        detail += w * m[" << i << "] * texture2DArray(oe_detail_tex, vec3(oe_detail_tc*" << ofx << ".0," << i << ")); \n";
                }
            }
        }
        else
        {
            for(unsigned i=0; i<numTextures; ++i)
            {
                buf <<
                    "        detail += m[" << i << "] * texture2DArray(oe_detail_tex, vec3(oe_detail_tc," << i << ")); \n";
            }
        }

        buf <<
            //"        vec3 luminosity = vec3(detail.r*0.2125 + detail.g*0.7154 + detail.b*0.0721);\n"
            "        color = mix(color, detail, oe_detail_intensity * oe_detail_atten_factor);\n"
            "    } \n"
            "} \n";

        std::string r;
        r = buf.str();
        return r;
    }
}


DetailTexture::DetailTexture() :
TerrainEffect(),
_startLOD    ( 10 ),
_intensity   ( 1.0f ),
_scale       ( 1.0f ),
_attenuationDistance( FLT_MAX ),
_octaves     ( 1 )
{
    init();
}

DetailTexture::DetailTexture(const Config& conf, const Map* map) :
TerrainEffect(),
_startLOD    ( 10 ),
_intensity   ( 1.0f ),
_scale       ( 1.0f ),
_attenuationDistance( FLT_MAX ),
_octaves     ( 1 )
{
    mergeConfig(conf);

    if ( map )
    {
        if ( _maskLayerName.isSet() )
        {
            _maskLayer = map->getImageLayerByName(*_maskLayerName);
        }

        _dbOptions = Registry::instance()->cloneOrCreateOptions(map->getDBOptions());
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );
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

    _attenuationDistanceUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_detail_attenuation_distance");
    _attenuationDistanceUniform->set( _attenuationDistance.get() );
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
DetailTexture::setAttenuationDistance(float value)
{
    _attenuationDistance = osg::clampAbove(value, 1.0f);
    _attenuationDistanceUniform->set( _attenuationDistance.get() );
}

void
DetailTexture::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        if ( !_texture.valid() )
        {
            _texture = new osg::Texture2DArray();
            _texture->setTextureSize(1024, 1024, _textures.size());
            _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
            _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
            _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
            _texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            _texture->setResizeNonPowerOfTwoHint( false );

            for(unsigned i=0; i<_textures.size(); ++i)
            {
                const TextureSource& ts = _textures[i];
                osg::ref_ptr<osg::Image> image = URI(ts._url).getImage(_dbOptions.get());
                if ( image->s() != 1024 || image->t() != 1024 )
                {
                    osg::ref_ptr<osg::Image> imageResized;
                    ImageUtils::resizeImage( image.get(), 1024, 1024, imageResized );
                    _texture->setImage( i, imageResized.get() );
                }
                else
                {
                    _texture->setImage( i, image.get() );
                }
            }
        }

        osg::StateSet* stateset = engine->getOrCreateStateSet();

        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_unit) )
        {
            _samplerUniform = stateset->getOrCreateUniform( "oe_detail_tex", osg::Uniform::SAMPLER_2D_ARRAY );
            _samplerUniform->set( _unit );
            stateset->setTextureAttribute( _unit, _texture.get() ); // don't use "..andModes"
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
        stateset->addUniform( _attenuationDistanceUniform.get() );

        std::string fs = generateFragmentShader( _textures.size(), _octaves.value() );
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_detail_vertex",   vs, ShaderComp::LOCATION_VERTEX_VIEW );
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
        stateset->removeUniform( _attenuationDistanceUniform.get() );

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
    conf.getIfSet( "attenuation_distance", _attenuationDistance );
    conf.getIfSet( "mask_layer", _maskLayerName );
    conf.getIfSet( "octaves",    _octaves );

    ConfigSet textures = conf.child("textures").children("texture");
    for(ConfigSet::iterator i = textures.begin(); i != textures.end(); ++i)
    {
        _textures.push_back(TextureSource());
        _textures.back()._tag = i->value("tag");
        _textures.back()._url = i->value("url");
    }
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
    conf.addIfSet( "attenuation_distance", _attenuationDistance );
    conf.addIfSet( "mask_layer", layername );
    conf.addIfSet( "octaves",    _octaves );

    if ( _textures.size() > 0 )
    {
        Config textures("textures");
        for(std::vector<TextureSource>::const_iterator i = _textures.begin(); i != _textures.end(); ++i )
        {
            Config texture("texture");
            texture.set("tag", i->_tag);
            texture.set("url", i->_url);
        }
    }

    return conf;
}

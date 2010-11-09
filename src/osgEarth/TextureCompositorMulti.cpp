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
#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorMultiTexture] "

//------------------------------------------------------------------------

namespace
{
    static osg::Shader*
    s_createTextureVertexShader( int maxLayersToRender )
    {
        std::stringstream buf;

        buf << "void osgearth_vert_texture( in vec3 position, in vec3 normal ) \n"
            << "{ \n";

        for(int i=0; i<maxLayersToRender; ++i )
        {
            buf << "    gl_TexCoord["<< i <<"] = gl_MultiTexCoord"<< i << "; \n";
        }
            
        buf << "} \n";

        std::string str = buf.str();
        return new osg::Shader( osg::Shader::VERTEX, str );
    }

    static osg::Shader*
    s_createTextureFragShaderFunction( const TextureLayout& layout, int maxLayersToRender )
    {
        const TextureSlotVector& slots = layout.getTextureSlots();
        const RenderOrderVector& order = layout.getRenderOrder();

        std::stringstream buf;

        buf << "#version 120 \n"
            << "uniform float[] osgearth_imagelayer_opacity; \n"
            << "uniform bool[]  osgearth_imagelayer_enabled; \n"
            << "uniform float[] osgearth_imagelayer_range; \n"
            << "uniform float   osgearth_imagelayer_attenuation; \n"
            << "varying float   osgearth_range; \n";

        buf << "uniform sampler2D ";
        for( unsigned int i=0; i<order.size(); ++i )
            buf << "tex"<< order[i] << (i+1 < order.size()? "," : ";");
        buf << "\n";

        buf << "vec4 osgearth_frag_texture(void) \n"
            << "{ \n"
            << "    vec3 color = vec3(1,1,1); \n"
            << "    vec4 texel; \n"
            << "    float dmin, dmax, atten_min, atten_max; \n";

        for( unsigned int i=0; i<order.size(); ++i )
        {
            int slot = order[i];
            int q = 2 * i;
            int r = 4 * slot;

            buf << "    if (osgearth_imagelayer_enabled["<< i << "]) { \n"
                << "        dmin = osgearth_range - osgearth_imagelayer_range["<< q << "]; \n"
                << "        dmax = osgearth_range - osgearth_imagelayer_range["<< q+1 <<"]; \n"
                << "        if (dmin >= 0 && dmax <= 0.0) { \n"
                << "            atten_max = -clamp( dmax, -osgearth_imagelayer_attenuation, 0 ) / osgearth_imagelayer_attenuation; \n"
                << "            atten_min =  clamp( dmin, 0, osgearth_imagelayer_attenuation ) / osgearth_imagelayer_attenuation; \n"
                << "            texel = texture2D(tex" << slot << ", gl_TexCoord["<< slot <<"].st); \n"
                << "            color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity[" << i << "] * atten_max * atten_min); \n"
                << "        } \n"
                << "    } \n";
        }

        buf << "    return vec4(color,1); \n"
            << "} \n";

        std::string str = buf.str();
        //OE_INFO << std::endl << str;
        return new osg::Shader( osg::Shader::FRAGMENT, str );
    }
}

//------------------------------------------------------------------------

namespace
{
#if 0
    static void s_setTexture( const GeoImage& i, int texUnit, bool addUniform, osg::StateSet* stateSet )
    {
        osg::Texture2D* texture = new osg::Texture2D( i.getImage() );
        int texWidth = i.getImage()->s();
        int texHeight = i.getImage()->t();

        // configure the mipmapping 
        texture->setMaxAnisotropy(16.0f);
        texture->setResizeNonPowerOfTwoHint(false);
        texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
        bool powerOfTwo = texWidth > 0 && (texWidth & (texWidth - 1)) && texHeight > 0 && (texHeight & (texHeight - 1));
        if ( powerOfTwo )
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        else
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );

        // configure the wrapping
        texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);

        // populate the stateset
        stateSet->setTextureAttributeAndModes( texUnit, texture, osg::StateAttribute::ON );

        if ( addUniform )
        {
            std::stringstream buf;
            buf << "tex" << texUnit;
            std::string name = buf.str();
            stateSet->addUniform( new osg::Uniform( name.c_str(), texUnit ) );
            // NOTE: "tex"+texUnit doesn't work.
        }
    }
#endif

    static osg::Texture2D*
    s_getTexture( osg::StateSet* stateSet, UID layerUID, const TextureLayout& layout )
    {
        int slot = layout.getSlot( layerUID );
        if ( slot < 0 )
            return 0L;

        osg::Texture2D* tex = static_cast<osg::Texture2D*>(
            stateSet->getTextureAttribute( slot, osg::StateAttribute::TEXTURE ) );

        if ( !tex )
        {
            tex = new osg::Texture2D();

            // configure the wrapping
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );

            stateSet->setTextureAttributeAndModes( slot, tex, osg::StateAttribute::ON );
            
            // install the slot attribute
            std::stringstream buf;
            buf << "tex" << slot;
            std::string name = buf.str();
            stateSet->addUniform( new osg::Uniform( name.c_str(), slot ) );
        }

        return tex;
    }
}

//------------------------------------------------------------------------

TextureCompositorMultiTexture::TextureCompositorMultiTexture( bool useGPU ) :
_useGPU( useGPU )
{
    //nop
}

#if 0
osg::StateSet*
TextureCompositorMultiTexture::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();
    if ( layerImages.size() < 1 )
        return stateSet;

    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i )
    {
        s_setTexture( *i, i-layerImages.begin(), _useGPU, stateSet );
    }

    return stateSet;
}
#endif

void
TextureCompositorMultiTexture::applyLayerUpdate(osg::StateSet* stateSet,
                                                UID layerUID,
                                                const GeoImage& preparedImage,
                                                const GeoExtent& tileExtent,
                                                const TextureLayout& layout ) const
{
    osg::Texture2D* tex = s_getTexture( stateSet, layerUID, layout );
    if ( tex )
    {
        tex->setImage( preparedImage.getImage() );

        // recalculate mipmapping filters:
        int texWidth  = tex->getImage()->s();
        int texHeight = tex->getImage()->t();
        bool powerOfTwo = texWidth > 0 && (texWidth & (texWidth - 1)) && texHeight > 0 && (texHeight & (texHeight - 1));
        if ( powerOfTwo )
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        else
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }
}


#if 0
    osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(
        stateSet->getTextureAttribute( layerNum, osg::StateAttribute::TEXTURE ) );

    if ( texture )
    {
        texture->setImage( preparedImage.getImage() );

        // recalculate mipmapping:
        int texWidth  = texture->getImage()->s();
        int texHeight = texture->getImage()->t();
        bool powerOfTwo = texWidth > 0 && (texWidth & (texWidth - 1)) && texHeight > 0 && (texHeight & (texHeight - 1));
        if ( powerOfTwo )
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        else
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }
    else
    {
        s_setTexture( preparedImage, layerNum, _useGPU, stateSet );
    }
}
#endif


void 
TextureCompositorMultiTexture::updateMasterStateSet(osg::StateSet* stateSet,
                                                    const TextureLayout& layout ) const
{
    int maxLayers = layout.getMaxUsedSlot() + 1;

    if ( _useGPU )
    {
        // Validate against the max number of GPU texture units:
        if ( maxLayers > Registry::instance()->getCapabilities().getMaxGPUTextureUnits() )
        {
            maxLayers = Registry::instance()->getCapabilities().getMaxGPUTextureUnits();
            OE_WARN << LC
                << "Warning! You have exceeded the number of texture units available on your GPU ("
                << maxLayers << "). Consider using another compositing mode."
                << std::endl;
        }

        VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(osg::StateAttribute::PROGRAM) );
        if ( maxLayers > 0 )
        {
            vp->setShader( "osgearth_frag_texture", s_createTextureFragShaderFunction(layout, maxLayers) );
            vp->setShader( "osgearth_vert_texture", s_createTextureVertexShader(maxLayers) );
        }
        else
        {
            vp->removeShader( "osgearth_frag_texture", osg::Shader::FRAGMENT );
            vp->removeShader( "osgearth_vert_texture", osg::Shader::VERTEX );
        }
    }

    else
    {
        // Validate against the maximum number of textures available in FFP mode.
        if ( maxLayers > Registry::instance()->getCapabilities().getMaxFFPTextureUnits() )
        {
            maxLayers = Registry::instance()->getCapabilities().getMaxFFPTextureUnits();
            OE_WARN << LC << 
                "Warning! You have exceeded the number of texture units available in fixed-function pipeline "
                "mode on your graphics hardware (" << maxLayers << "). Consider using another "
                "compositing mode." << std::endl;
        }

        // FFP multitexturing requires that we set up a series of TexCombine attributes:
        if (maxLayers == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateSet->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (maxLayers >= 2)
        {
            //Blend together the colors and accumulate the alpha values of textures 0 and 1 on unit 0
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateSet->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
            }


            //For textures 2 and beyond, blend them together with the previous
            //Add the alpha values of this unit and the previous unit
            for (int unit = 1; unit < maxLayers-1; ++unit)
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateSet->setTextureAttributeAndModes(unit, texenv, osg::StateAttribute::ON);
            }

            //Modulate the colors to get proper lighting on the last unit
            //Keep the alpha results from the previous stage
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::REPLACE);

                texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                stateSet->setTextureAttributeAndModes(maxLayers-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
}

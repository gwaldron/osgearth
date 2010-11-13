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

        buf << "void osgearth_vert_setupTexturing() \n"
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
        const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();
        const TextureLayout::RenderOrderVector& order = layout.getRenderOrder();

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

        buf << "void osgearth_frag_applyTexturing( inout vec4 color ) \n"
            << "{ \n"
            << "    vec3 color3 = color.rgb; \n"
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
                << "            color3 = mix(color3, texel.rgb, texel.a * osgearth_imagelayer_opacity[" << i << "] * atten_max * atten_min); \n"
                << "        } \n"
                << "    } \n";
        }

        buf << "    color = vec4(color3,color.a); \n"
            << "} \n";

        std::string str = buf.str();
        //OE_INFO << std::endl << str;
        return new osg::Shader( osg::Shader::FRAGMENT, str );
    }
}

//------------------------------------------------------------------------

namespace
{
    static osg::Texture2D*
    s_getTexture( osg::StateSet* stateSet, UID layerUID, const TextureLayout& layout, bool lodBlending )
    {
        int slot = layout.getSlot( layerUID );
        if ( slot < 0 )
            return 0L;

        osg::Texture2D* tex = static_cast<osg::Texture2D*>(
            stateSet->getTextureAttribute( slot, osg::StateAttribute::TEXTURE ) );

        if ( !tex )
        {
            tex = new osg::Texture2D();

            // configure the mipmapping

            // only enable anisotropic filtering if we are NOT using mipmap blending.
            tex->setMaxAnisotropy( lodBlending ? 1.0f : 16.0f );

            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );

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

TextureCompositorMultiTexture::TextureCompositorMultiTexture( bool useGPU, bool lodBlending ) :
_useGPU( useGPU ),
_lodBlending( lodBlending )
{
    //nop
}

#if 0
GeoImage
TextureCompositorMultiTexture::prepareImage( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
{
    osg::ref_ptr<osg::Image> image = layerImage.getImage();

    // Because all tex2darray layers must be identical in format, let's use RGBA.
    if ( image->getPixelFormat() != GL_RGBA )
        image = ImageUtils::convertToRGBA( image.get() );

    // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
    if ( image->s() != 256 || image->t() != 256 )
        image = ImageUtils::resizeImage( image.get(), 256, 256 );

    //Make sure that the internal texture format is always set to GL_RGBA
    image->setInternalTextureFormat( GL_RGBA );
    
    // Failure to do this with a Texture2DArray will result in texture corruption if we are 
    // updating layers (like in sequential mode).
    image->setDataVariance( osg::Object::DYNAMIC );

    return GeoImage( image.get(), layerImage.getExtent() );
}
#endif

void
TextureCompositorMultiTexture::applyLayerUpdate(osg::StateSet* stateSet,
                                                UID layerUID,
                                                const GeoImage& preparedImage,
                                                const GeoExtent& tileExtent,
                                                const TextureLayout& layout ) const
{
    osg::Texture2D* tex = s_getTexture( stateSet, layerUID, layout, _lodBlending );
    if ( tex )
    {
        tex->setImage( preparedImage.getImage() );
    }
}

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
            vp->setShader( "osgearth_frag_applyTexturing", s_createTextureFragShaderFunction(layout, maxLayers) );
            vp->setShader( "osgearth_vert_setupTexturing", s_createTextureVertexShader(maxLayers) );
        }
        else
        {
            vp->removeShader( "osgearth_frag_applyTexturing", osg::Shader::FRAGMENT );
            vp->removeShader( "osgearth_vert_setupTexturing", osg::Shader::VERTEX );
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

void
TextureCompositorMultiTexture::applyResourcePolicy(const ResourcePolicy& rp,
                                                   TextureLayout& layout ) const
{
    // multitexture mode maps slots to texture image units:
    layout.setReservedSlots( rp.getReservedTextureImageUnits() );
}

osg::Shader*
TextureCompositorMultiTexture::createSamplerFunction(UID layerUID,
                                                     const std::string& functionName,
                                                     osg::Shader::Type type,
                                                     const TextureLayout& layout ) const
{
    osg::Shader* result = 0L;

    int slot = layout.getSlot( layerUID );
    if ( slot >= 0 )
    {
        std::stringstream buf;

        buf << "uniform sampler2D tex"<< slot << "; \n"
            << "vec4 " << functionName << "() \n"
            << "{ \n";

        if ( type == osg::Shader::VERTEX )
            buf << "    return texture2D(tex"<< slot << ", gl_MultiTexCoord"<< slot <<".st); \n";
        else
            buf << "    return texture2D(tex"<< slot << ", gl_TexCoord["<< slot << "].st); \n";

        buf << "} \n";

        std::string str = buf.str();
        result = new osg::Shader( type, str );
    }
    return result;
}

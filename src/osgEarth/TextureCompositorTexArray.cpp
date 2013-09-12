/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/TextureCompositorTexArray>

// only in newer OSG versions.
#if OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

#include <sstream>

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osgEarth/SparseTexture2DArray>
#include <osgEarth/ShaderUtils>
#include <osgEarth/TileKey>
#include <osgEarth/Capabilities>

using namespace osgEarth;

#define LC "[TextureCompositorTexArray] "


//------------------------------------------------------------------------

namespace
{
    static osg::Shader*
    s_createTextureVertSetupShaderFunction( const TextureLayout& layout )
    {
        std::stringstream buf;
       
        const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();

        buf << "#version 130 \n"
            << "varying vec4 osg_FrontColor; \n"
            << "varying vec4 osg_FrontSecondaryColor; \n"

            << "void osgearth_vert_setupColoring() \n"
            << "{ \n"
            << "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
            << "    osg_FrontColor = gl_Color; \n"
            << "    osg_FrontSecondaryColor = vec4(0.0); \n"
            << "} \n";

        std::string str;
        str = buf.str();
        return new osg::Shader( osg::Shader::VERTEX, str );
    }

    static osg::Shader*
    s_createTextureFragShaderFunction( const TextureLayout& layout, bool blending, float blendTime )
    {
        int numSlots = layout.getMaxUsedSlot() + 1;

        std::stringstream buf;

        buf << "#version 130 \n"
            << "#extension GL_EXT_gpu_shader4 : enable \n"
            << "varying vec4 osg_FrontColor; \n"
            << "varying vec4 osg_FrontSecondaryColor; \n";
            

        if ( numSlots <= 0 )
        {
            // No textures : create a no-op shader
            buf << "void osgearth_frag_applyColoring( inout vec4 color ) \n"
                << "{ \n"
                << "    color = osg_FrontColor; \n"
                << "} \n";
        }
        else
        {
            if ( blending )
            {
                buf << "#extension GL_ARB_shader_texture_lod : enable \n"
                    << "uniform float osgearth_SlotStamp[ " << numSlots << "]; \n"
                    << "uniform float osg_FrameTime;\n"
                    << "uniform float osgearth_LODRangeFactor;\n";
            }

            buf << "uniform sampler2DArray tex0; \n";
            
            if ( blending )
                buf << "uniform sampler2DArray tex1;\n";

            buf << "uniform float region[ " << 8*numSlots << "]; \n"
                << "uniform float osgearth_ImageLayerOpacity[" << numSlots << "]; \n"
                << "uniform bool  osgearth_ImageLayerVisible[" << numSlots << "]; \n"
                << "uniform float osgearth_ImageLayerRange[" << 2*numSlots << "]; \n"
                << "uniform float osgearth_ImageLayerAttenuation; \n"
                << "uniform float osgearth_CameraElevation; \n"

                << "void osgearth_frag_applyColoring( inout vec4 color ) \n"
                << "{ \n"
                << "    vec3 color3 = color.rgb; \n"
                << "    float u, v, dmin, dmax, atten_min, atten_max, age; \n"
                << "    vec4 texel; \n";

            const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();
            const TextureLayout::RenderOrderVector& order = layout.getRenderOrder();

            for( unsigned int i = 0; i < order.size(); ++i )
            {
                int slot = order[i];
                int q = 2 * i;
                int r = 8 * slot;
                UID uid = slots[slot];

                buf << "    if (osgearth_ImageLayerVisible["<< i << "]) \n"
                    << "    { \n"
                    << "        u = region["<< r <<"] + (region["<< r+2 <<"] * gl_TexCoord[0].s); \n"
                    << "        v = region["<< r+1 <<"] + (region["<< r+3 <<"] * gl_TexCoord[0].t); \n"
                    << "        dmin = osgearth_CameraElevation - osgearth_ImageLayerRange["<< q << "]; \n"
                    << "        dmax = osgearth_CameraElevation - osgearth_ImageLayerRange["<< q+1 <<"]; \n"
                    << "        if (dmin >= 0 && dmax <= 0.0) \n"
                    << "        { \n"
                    << "            atten_max = -clamp( dmax, -osgearth_ImageLayerAttenuation, 0 ) / osgearth_ImageLayerAttenuation; \n"
                    << "            atten_min =  clamp( dmin, 0, osgearth_ImageLayerAttenuation ) / osgearth_ImageLayerAttenuation; \n";

                if ( layout.isBlendingEnabled(uid) )
                {
                    float invBlendTime = 1.0f/blendTime;

                    buf << "            age = "<< invBlendTime << " * min( "<< blendTime << ", osg_FrameTime - osgearth_SlotStamp[" << slot << "] ); \n"
                        << "            age = clamp(age, 0.0, 1.0);\n"
                        << "            float pu, pv;\n"
                        << "            pu = region["<< r+4 <<"] + (region["<< r+6 <<"] * gl_TexCoord[0].s); \n"
                        << "            pv = region["<< r+5 <<"] + (region["<< r+7 <<"] * gl_TexCoord[0].t); \n"

                        << "            vec3 texCoord = vec3(pu, pv, " << slot <<");\n;\n"
                        << "            vec4 texel0 = texture2DArray( tex0, vec3(u, v, " << slot << ") );\n"
                        << "            vec4 texel1 = texture2DArray( tex1, vec3(pu, pv, " << slot << ") );\n"
                        << "            float mixval = age * osgearth_LODRangeFactor;\n"
                        
                        // pre-multiply alpha before mixing:
                        << "            texel0.rgb *= texel0.a; \n"
                        << "            texel1.rgb *= texel1.a; \n"
                        << "            texel = mix(texel1, texel0, mixval); \n"

                        // revert to non-pre-multiplies alpha (assumes openGL state uses non-pre-mult alpha)
                        << "            if (texel.a > 0.0) { \n"
                        << "                texel.rgb /= texel.a; \n"
                        << "            } \n";
                }
                else
                {
                    buf << "            texel = texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n";
                }

                buf << "            color3 = mix(color3, texel.rgb, texel.a * osgearth_ImageLayerOpacity["<< i <<"] * atten_max * atten_min); \n"
                    << "        } \n"
                    << "    } \n";
            }

            buf << "    color = vec4(color3.rgb, color.a); \n"
                << "} \n";
        }

        std::string str;
        str = buf.str();
        return new osg::Shader( osg::Shader::FRAGMENT, str );
    }
}

//------------------------------------------------------------------------

namespace
{
    osg::Texture2DArray*
    s_getTexture( osg::StateSet* stateSet, const TextureLayout& layout,
                  int unit, unsigned textureSize, osg::Texture::FilterMode minFilter, osg::Texture::FilterMode magFilter )
    {
        osg::Texture2DArray* tex = static_cast<osg::Texture2DArray*>(
            stateSet->getTextureAttribute( unit, osg::StateAttribute::TEXTURE ) );

        // if the texture array doesn't exist, create it anew.
        if ( !tex )
        {
            tex = new SparseTexture2DArray();
            tex->setSourceFormat( GL_RGBA );
            tex->setInternalFormat( GL_RGBA8 );
            tex->setTextureWidth( textureSize );
            tex->setTextureHeight( textureSize );

            // configure the mipmapping
            tex->setMaxAnisotropy(16.0f);
            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
            tex->setFilter( osg::Texture::MIN_FILTER, minFilter );

            // configure the wrapping
            tex->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
            tex->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);

            stateSet->setTextureAttribute( unit, tex, osg::StateAttribute::ON );
        }

        // grow the texture array if necessary.
        int requiredDepth = layout.getMaxUsedSlot() + 1;
        if ( tex->getTextureDepth() < requiredDepth )
            tex->setTextureDepth( requiredDepth );

        const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();

        // null out any empty slots (to save memory, i guess)
        for( int i=0; i < tex->getTextureDepth(); ++i )
        {
            if ( i < (int)slots.size() && slots[i] < 0 )
                tex->setImage( i, 0L );
        }

        return tex;
    }
    
    osg::Uniform* ensureSampler(osg::StateSet* ss, int unit)
    {
        std::stringstream sstream;
        sstream << "tex" << unit;
        std::string str;
        str = sstream.str();
        osg::ref_ptr<osg::Uniform> sampler = ss->getUniform(str);
        int samplerUnit = -1;
        if (sampler.valid() && sampler->getType() == osg::Uniform::SAMPLER_2D_ARRAY)
            sampler->get(samplerUnit);
        if (samplerUnit == -1 || samplerUnit != unit)
        {
            sampler = new osg::Uniform(osg::Uniform::SAMPLER_2D_ARRAY, str);
            sampler->set(unit);
            ss->addUniform(sampler.get());
        }
        return sampler.get();
    }

    void assignImage(osg::Texture2DArray* texture, int slot, osg::Image* image)
    {
        // We have to dirty() the image because otherwise the texture2d
        // array implementation will not recognize it as new data.
        image->dirty();
        texture->setImage( slot, image );

        if (ImageUtils::isPowerOfTwo( image ) && !(!image->isMipmap() && ImageUtils::isCompressed(image)))
        {
            if ( texture->getFilter(osg::Texture::MIN_FILTER) != osg::Texture::LINEAR_MIPMAP_LINEAR )
                texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        }
        else if ( texture->getFilter(osg::Texture::MIN_FILTER) != osg::Texture::LINEAR )
        {
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
        }
    }

    void getImageTransform(
        const GeoExtent& tileExtent,
        const GeoExtent& imageExtent,
        osg::Vec4&       transform)
    {
        if (!tileExtent.isValid() || !imageExtent.isValid())
            return;

        transform[0] = (tileExtent.xMin() - imageExtent.xMin()) / imageExtent.width();
        transform[1] = (tileExtent.yMin() - imageExtent.yMin()) / imageExtent.height();
        transform[2]  = tileExtent.width() / imageExtent.width();
        transform[3]  = tileExtent.height() / imageExtent.height();
    }
}

//------------------------------------------------------------------------

bool
TextureCompositorTexArray::isSupported()
{
    const Capabilities& caps = osgEarth::Registry::instance()->getCapabilities();
    return caps.supportsGLSL(1.30f) && caps.supportsTextureArrays();
}

TextureCompositorTexArray::TextureCompositorTexArray( const TerrainOptions& options ) :
_lodTransitionTime( *options.lodTransitionTime() ),
_minFilter        ( *options.minFilter() ),
_magFilter        ( *options.magFilter() )
{
    //nop
}

GeoImage
TextureCompositorTexArray::prepareImage( const GeoImage& layerImage, const GeoExtent& tileExtent, unsigned textureSize ) const
{
    const osg::Image* image = layerImage.getImage();
    if (!image)
        return GeoImage::INVALID;

    if (image->getPixelFormat() != GL_RGBA ||
        image->getInternalTextureFormat() != GL_RGBA8 ||
        image->s() != textureSize ||
        image->t() != textureSize )
    {
        // Because all tex2darray layers must be identical in format, let's use RGBA.
        osg::ref_ptr<osg::Image> newImage = ImageUtils::convertToRGBA8( image );
        
        // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
        if ( image->s() != textureSize || image->t() != textureSize )
        {
            osg::ref_ptr<osg::Image> resizedImage;
            if ( ImageUtils::resizeImage( newImage.get(), textureSize, textureSize, resizedImage ) )
                newImage = resizedImage.get();
        }

        return GeoImage( newImage.get(), layerImage.getExtent() );
    }
    else
    {
        return layerImage;
    }
}

void
TextureCompositorTexArray::applyLayerUpdate(osg::StateSet*       stateSet,
                                            UID                  layerUID,
                                            const GeoImage&      preparedImage,
                                            const TileKey&       tileKey,
                                            const TextureLayout& layout,
                                            osg::StateSet*       parentStateSet) const
{
    GeoExtent tileExtent(tileKey.getExtent());
    int slot = layout.getSlot( layerUID );
    if ( slot < 0 )
        return; // means the layer no longer exists

    // access the texture array, creating or growing it if necessary:
    osg::Texture2DArray* texture = s_getTexture( stateSet, layout, 0,
                                                 textureSize(), _minFilter, _magFilter);
    ensureSampler( stateSet, 0 );
    // assign the new image at the proper position in the texture array.
    osg::Image* image = preparedImage.getImage();
    assignImage(texture, slot, image);
    
    // update the region uniform to reflect the geo extent of the image:
    const GeoExtent& imageExtent = preparedImage.getExtent();
    osg::Vec4 tileTransform;
    getImageTransform(tileExtent, imageExtent, tileTransform);

    // access the region uniform, creating or growing it if necessary:
    ArrayUniform regionUni( "region", osg::Uniform::FLOAT, stateSet, layout.getMaxUsedSlot()+1 );
    if ( regionUni.isValid() )
    {
        int layerOffset = slot * 8;
        for (int i = 0; i < 4; ++i)
            regionUni.setElement( layerOffset + i, tileTransform[i]);
        //region->dirty();
    }
    
    if ( layout.isBlendingEnabled( layerUID ) && regionUni.isValid() )
    {
        osg::Uniform* secondarySampler = ensureSampler( stateSet, 1 );
        osg::Texture2DArray* parentTexture = 0;
        const unsigned parentLayerOffset = slot * 8 + 4;
        if ( parentStateSet )
        {
            ArrayUniform parentRegion( "region", osg::Uniform::FLOAT, parentStateSet, layout.getMaxUsedSlot()+1 );

            //osg::Uniform* parentRegion = s_getRegionUniform( parentStateSet,
            //                                                 layout );
            GeoExtent parentExtent(tileKey.createParentKey().getExtent());
            float widthRatio, heightRatio;
            parentRegion.getElement(slot * 8 + 2, widthRatio);
            parentRegion.getElement(slot * 8 + 3, heightRatio);
            float parentImageWidth =  parentExtent.width() / widthRatio;
            float parentImageHeight = parentExtent.height() / heightRatio;
            float xRatio, yRatio;
            parentRegion.getElement(slot * 8, xRatio);
            parentRegion.getElement(slot * 8 + 1, yRatio);
            float ParentImageXmin = parentExtent.xMin() - xRatio * parentImageWidth;
            float ParentImageYmin = parentExtent.yMin() - yRatio * parentImageHeight;
            regionUni.setElement(parentLayerOffset,
                               static_cast<float>((tileExtent.xMin() - ParentImageXmin) / parentImageWidth));
            regionUni.setElement(parentLayerOffset + 1,
                               static_cast<float>((tileExtent.yMin() - ParentImageYmin) / parentImageHeight));
            regionUni.setElement(parentLayerOffset + 2,
                               static_cast<float>(tileExtent.width() / parentImageWidth));
            regionUni.setElement(parentLayerOffset + 3,
                               static_cast<float>(tileExtent.height() / parentImageHeight));
            //regionUni.dirty();
            parentTexture = static_cast<osg::Texture2DArray*>(parentStateSet->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
        }
        else
        {
            // setting the parent transform values to -1 disabled blending for this layer. #hack -gw
            for (int i = 0; i < 4; ++i)
                regionUni.setElement(parentLayerOffset + i, tileTransform[i]);
        }

        if (parentTexture)
            stateSet->setTextureAttribute(1, parentTexture, osg::StateAttribute::ON);
        else
            secondarySampler->set(0);

        // update the timestamp on the image layer to support fade-in blending.
        float now = (float)osg::Timer::instance()->delta_s( osg::Timer::instance()->getStartTick(), osg::Timer::instance()->tick() );
        ArrayUniform stampUniform( "osgearth_SlotStamp", osg::Uniform::FLOAT, stateSet, layout.getMaxUsedSlot()+1 );
        stampUniform.setElement( slot, now );
    }
}

void
TextureCompositorTexArray::updateMasterStateSet( osg::StateSet* stateSet, const TextureLayout& layout ) const
{
    VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(VirtualProgram::SA_TYPE) );

    vp->setShader(
        "osgearth_vert_setupColoring",
        s_createTextureVertSetupShaderFunction(layout),
        osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );

    vp->setShader( 
        "osgearth_frag_applyColoring", 
        s_createTextureFragShaderFunction(layout, true, _lodTransitionTime ),
        osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
}

osg::Shader*
TextureCompositorTexArray::createSamplerFunction(UID layerUID,
                                                 const std::string& functionName,
                                                 osg::Shader::Type type,
                                                 const TextureLayout& layout ) const
{
    osg::Shader* result = 0L;

    int slot = layout.getSlot( layerUID );
    if ( slot >= 0 )
    {
        int r = 8 * slot;

        std::string texCoord = 
            type == osg::Shader::VERTEX ? "gl_MultiTexCoord0" : 
            "gl_TexCoord[0]";

        std::stringstream buf;

        buf << "#version 130 \n"
            << "#extension GL_EXT_gpu_shader4 : enable \n"

            << "uniform sampler2DArray tex0; \n"
            << "uniform float[] region; \n"

            << "vec4 " << functionName << "() \n"
            << "{ \n"
            << "    float u = region["<< r <<"] + (region["<< r+2 <<"] * "<< texCoord <<".s); \n"
            << "    float v = region["<< r+1 <<"] + (region["<< r+3 <<"] * "<< texCoord <<".t); \n"
            << "    return texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n"
            << "} \n";

        std::string str;
        str = buf.str();
        result = new osg::Shader( type, str );
    }
    return result;
}

//------------------------------------------------------------------------

#endif // OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

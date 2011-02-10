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
#include <osgEarth/TextureCompositorTexArray>

// only in newer OSG versions.
#if OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osgEarth/SparseTexture2DArray>

using namespace osgEarth;

#define LC "[TextureCompositorTexArray] "


//------------------------------------------------------------------------

namespace
{
    static osg::Shader*
    s_createTextureFragShaderFunction( const TextureLayout& layout, bool blending, float blendTime )
    {
        int numSlots = layout.getMaxUsedSlot() + 1;

        std::stringstream buf;

        buf << "#version 130 \n"
            << "#extension GL_EXT_gpu_shader4 : enable \n";
        

        if ( blending )
        {
            buf << "#extension GL_ARB_shader_texture_lod : enable \n"
                << "uniform float osgearth_SlotStamp[ " << numSlots << "]; \n"
                << "uniform float osg_FrameTime; \n";
        }

        buf << "uniform sampler2DArray tex0; \n"
            << "uniform float region[ " << 4*numSlots << "]; \n"
            << "uniform float osgearth_ImageLayerOpacity[" << numSlots << "]; \n"
            << "uniform bool  osgearth_ImageLayerEnabled[" << numSlots << "]; \n"
            << "uniform float osgearth_ImageLayerRange[" << 2*numSlots << "]; \n"
            << "uniform float osgearth_ImageLayerAttenuation; \n"
            << "varying float osgearth_CameraRange; \n"

            << "void osgearth_frag_applyTexturing( inout vec4 color ) \n"
            << "{ \n"
            << "    vec3 color3 = color.rgb; \n"
            << "    float u, v, dmin, dmax, atten_min, atten_max, age; \n"
            << "    vec4 texel; \n";

        const TextureLayout::RenderOrderVector& order = layout.getRenderOrder();

        for( unsigned int i = 0; i < order.size(); ++i )
        {
            int slot = order[i];
            int q = 2 * i;
            int r = 4 * slot;

            buf << "    if (osgearth_ImageLayerEnabled["<< i << "]) { \n"
                << "        u = region["<< r <<"] + (region["<< r+2 <<"] * gl_TexCoord[0].s); \n"
                << "        v = region["<< r+1 <<"] + (region["<< r+3 <<"] * gl_TexCoord[0].t); \n"
                << "        dmin = osgearth_CameraRange - osgearth_ImageLayerRange["<< q << "]; \n"
                << "        dmax = osgearth_CameraRange - osgearth_ImageLayerRange["<< q+1 <<"]; \n"
                << "        if (dmin >= 0 && dmax <= 0.0) { \n"
                << "            atten_max = -clamp( dmax, -osgearth_ImageLayerAttenuation, 0 ) / osgearth_ImageLayerAttenuation; \n"
                << "            atten_min =  clamp( dmin, 0, osgearth_ImageLayerAttenuation ) / osgearth_ImageLayerAttenuation; \n";
                       
            if ( blending )
            {
                float invBlendTime = 1.0f/blendTime;

                buf << "            age = "<< invBlendTime << " * min( "<< blendTime << ", osg_FrameTime - osgearth_SlotStamp[" << slot << "] ); \n"
                    << "            if ( age < 1.0 ) \n"
                    << "                texel = texture2DArrayLod( tex0, vec3(u,v,"<< slot <<"), 1.0-age); \n"
                    << "            else \n"
                    << "                texel = texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n";
            }
            else
            {
                buf << "            texel = texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n";
            }
  
            buf << "            color3 = mix(color3, texel.rgb, texel.a * osgearth_ImageLayerOpacity["<< i <<"] * atten_max * atten_min); \n"
                << "        } \n"
                << "    } \n"
                ;
        }

        buf << "    color = vec4(color3.rgb, color.a); \n"
            << "} \n";

        std::string str = buf.str();
        return new osg::Shader( osg::Shader::FRAGMENT, str );
    }
}

//------------------------------------------------------------------------

namespace
{
    static osg::Texture2DArray*
    s_getTexture( osg::StateSet* stateSet, const TextureLayout& layout )
    {
        osg::Texture2DArray* tex = static_cast<osg::Texture2DArray*>(
            stateSet->getTextureAttribute( 0, osg::StateAttribute::TEXTURE ) );

        // if the texture array doesn't exist, create it anew.
        if ( !tex )
        {
            tex = new SparseTexture2DArray();
            tex->setSourceFormat( GL_RGBA );
            tex->setInternalFormat( GL_RGBA8 );
            tex->setTextureWidth( 256 );
            tex->setTextureHeight( 256 );
            
            // configure the mipmapping 
            tex->setMaxAnisotropy(16.0f);
            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );

            // configure the wrapping
            tex->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
            tex->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);

            stateSet->setTextureAttribute( 0, tex, osg::StateAttribute::ON );
            stateSet->getOrCreateUniform( "tex0", osg::Uniform::SAMPLER_2D_ARRAY )->set( 0 );
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

    static osg::Uniform*
    s_getRegionUniform( osg::StateSet* stateSet, const TextureLayout& layout )
    {
        osg::Uniform* region = stateSet->getUniform( "region" );

        // if the region-uniform doesn't exist, create it now
        if ( !region )
        {
            region = new osg::Uniform( osg::Uniform::FLOAT, "region", layout.getTextureSlots().size() * 4 );
            stateSet->addUniform( region );
        }

        // if the region exists but is too small, re-allocate it (cannot grow it) and copy over the old values
        else if ( region->getNumElements() < layout.getTextureSlots().size() * 4 )
        {            
            osg::Uniform* newRegion = new osg::Uniform( osg::Uniform::FLOAT, "region", layout.getTextureSlots().size() * 4 );
            for( unsigned int i=0; i<region->getNumElements(); ++i )
            {
                float value;
                region->getElement( i, value );
                newRegion->setElement( i, value );
            }

            stateSet->removeUniform( region );
            stateSet->addUniform( newRegion );
            region = newRegion;
        }

        return region;
    }
} // namespace

//------------------------------------------------------------------------

TextureCompositorTexArray::TextureCompositorTexArray( const TerrainOptions& options ) :
_lodBlending( *options.lodBlending() ),
_lodTransitionTime( *options.lodTransitionTime() )
{
    // validate
    if ( _lodBlending && _lodTransitionTime <= 0.0f )
    {
        _lodBlending = false;
        OE_WARN << LC << "Disabling LOD blending because transition time <= 0.0" << std::endl;
    }
}

GeoImage
TextureCompositorTexArray::prepareImage( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
{
    const osg::Image* image = layerImage.getImage();

    if (image->getPixelFormat() != GL_RGBA ||
        image->getInternalTextureFormat() != GL_RGBA8 ||
        image->s() != 256 ||
        image->t() != 256 )
    {
        // Because all tex2darray layers must be identical in format, let's use RGBA.
        osg::ref_ptr<osg::Image> newImage = ImageUtils::convertToRGBA8( image );
        
        // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
        if ( image->s() != 256 || image->t() != 256 )
        {
            osg::ref_ptr<osg::Image> resizedImage;
            if ( ImageUtils::resizeImage( newImage.get(), 256, 256, resizedImage ) )
                newImage = resizedImage.get();
        }

        return GeoImage( newImage.get(), layerImage.getExtent() );
    }
    else
    {
        return layerImage;
    }
    
    // NOTE: moved this into TileSource::getImage.
    // Failure to do this with a Texture2DArray will result in texture corruption if we are 
    // updating layers (like in sequential mode).
    //const_cast<osg::Image*>(image.get())->setDataVariance( osg::Object::DYNAMIC );
}

void
TextureCompositorTexArray::applyLayerUpdate(osg::StateSet* stateSet,
                                            UID layerUID,
                                            const GeoImage& preparedImage,
                                            const GeoExtent& tileExtent,
                                            const TextureLayout& layout ) const
{
    int slot = layout.getSlot( layerUID );
    if ( slot < 0 )
        return; // means the layer no longer exists

    // access the texture array, creating or growing it if necessary:
    osg::Texture2DArray* texture = s_getTexture( stateSet, layout );

    // assign the new image at the proper position in the texture array. We have to 
    // dirty() the image because otherwise the texture2d array implementation will not
    // recognize it as new data.
    osg::Image* image = preparedImage.getImage();
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
    
    // update the region uniform to reflect the geo extent of the image:
    const GeoExtent& imageExtent = preparedImage.getExtent();
    float xoffset = (tileExtent.xMin() - imageExtent.xMin()) / imageExtent.width();
    float yoffset = (tileExtent.yMin() - imageExtent.yMin()) / imageExtent.height();
    float xscale  = tileExtent.width() / imageExtent.width();
    float yscale  = tileExtent.height() / imageExtent.height();

    // access the region uniform, creating or growing it if necessary:
    osg::Uniform* region = s_getRegionUniform( stateSet, layout );
    if ( region )
    {
        int layerOffset = slot * 4;
        region->setElement( layerOffset,     xoffset );
        region->setElement( layerOffset + 1, yoffset );
        region->setElement( layerOffset + 2, xscale );
        region->setElement( layerOffset + 3, yscale );
        region->dirty();
    }
    
    if ( _lodBlending )
    {
        // update the timestamp on the image layer to support blending.
        osg::Uniform* stamp = stateSet->getUniform( "osgearth_SlotStamp" );
        if ( !stamp || stamp->getNumElements() < (unsigned int)layout.getMaxUsedSlot() + 1 )
        {
            stamp = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_SlotStamp", layout.getMaxUsedSlot()+1 );   
            stateSet->addUniform( stamp );
        }

        float now = (float)osg::Timer::instance()->delta_s( osg::Timer::instance()->getStartTick(), osg::Timer::instance()->tick() );
        stamp->setElement( slot, now );
    }
}

void
TextureCompositorTexArray::updateMasterStateSet( osg::StateSet* stateSet, const TextureLayout& layout ) const
{
    VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(osg::StateAttribute::PROGRAM) );

    vp->setShader( 
        "osgearth_frag_applyTexturing", 
        s_createTextureFragShaderFunction(layout, _lodBlending, _lodTransitionTime ) );
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
        int r = 4 * slot;

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

        std::string str = buf.str();
        result = new osg::Shader( type, str );
    }
    return result;
}

//------------------------------------------------------------------------

#endif // OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

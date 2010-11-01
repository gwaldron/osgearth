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
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osgEarth/SparseTexture2DArray>

using namespace osgEarth;

#define LC "[TextureCompositorTexArray] "


//------------------------------------------------------------------------

static osg::Shader*
s_createTextureFragShaderFunction( const TextureLayout& layout )
{
    std::stringstream buf;

    buf << "#version 130 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n"

        << "uniform sampler2DArray tex0; \n"
        << "uniform float[] region; \n"
        << "uniform float[] osgearth_imagelayer_opacity; \n"
        << "uniform bool[]  osgearth_imagelayer_enabled; \n"
        << "uniform float[] osgearth_imagelayer_range; \n"
        << "uniform float   osgearth_imagelayer_attenuation; \n"
        << "varying float osgearth_range; \n"

        << "vec4 osgearth_frag_texture(void) \n"
        << "{ \n"
        << "    vec3 color = vec3(1,1,1); \n"
        << "    float u, v, dmin, dmax, atten_min, atten_max; \n"
        << "    vec4 texel; \n";

    const RenderOrderVector& order = layout.getRenderOrder();

    for( int i = 0; i < order.size(); ++i )
    {
        int slot = order[i];
        int q = 2 * i;
        int r = 4 * slot;

        buf << "    if (osgearth_imagelayer_enabled["<< i << "]) { \n"
            << "        u = region["<< r <<"] + (region["<< r+2 <<"] * gl_TexCoord[0].s); \n"
            << "        v = region["<< r+1 <<"] + (region["<< r+3 <<"] * gl_TexCoord[0].t); \n"
            << "        dmin = osgearth_range - osgearth_imagelayer_range["<< q << "]; \n"
            << "        dmax = osgearth_range - osgearth_imagelayer_range["<< q+1 <<"]; \n"
            << "        if (dmin >= 0 && dmax <= 0.0) { \n"
            << "            atten_max = -clamp( dmax, -osgearth_imagelayer_attenuation, 0 ) / osgearth_imagelayer_attenuation; \n"
            << "            atten_min =  clamp( dmin, 0, osgearth_imagelayer_attenuation ) / osgearth_imagelayer_attenuation; \n"
            << "            texel = texture2DArray( tex0, vec3(u,v,"<< slot <<") ); \n"
            << "            color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i <<"] * atten_max * atten_min); \n"
            << "        } \n"
            << "    } \n"
            ;
    }

    buf << "    return vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    //OE_INFO << std::endl << str;
    return new osg::Shader( osg::Shader::FRAGMENT, str );
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
            tex->setInternalFormat( GL_RGBA );
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
            stateSet->addUniform( new osg::Uniform("tex0", 0) );
        }

        // grow the texture array if necessary.
        int requiredDepth = layout.getMaxUsedSlot() + 1;
        if ( tex->getTextureDepth() < requiredDepth )
            tex->setTextureDepth( requiredDepth );

        const TextureSlotVector& slots = layout.getTextureSlots();

        // null out any empty slots (to save memory, i guess)
        for( int i=0; i<tex->getTextureDepth(); ++i )
        {
            if ( i < slots.size() && slots[i] < 0 )
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
            for( int i=0; i<region->getNumElements(); ++i )
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
};

//------------------------------------------------------------------------

GeoImage
TextureCompositorTexArray::prepareImage( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
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
}

#if 0
osg::StateSet*
TextureCompositorTexArray::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();
    if ( layerImages.size() < 1 )
        return stateSet;

    osg::Texture2DArray* texture = new osg::Texture2DArray();

    int texWidth = 0, texHeight = 0;

    // find each image layer and create a region entry for it
    texture->setTextureDepth( layerImages.size() );
    texture->setInternalFormat( GL_RGBA );

    // this uniform will properly position the image within the tile (kind of like
    // texture coordinate generation parameters).
    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "region", layerImages.size() * 4 );

    int layerNum = 0, u = 0;
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage& geoImage = *i;
        GeoImage preparedImage = prepareLayerUpdate( geoImage, tileExtent );
        osg::Image* image = preparedImage.getImage();

        // add the layer image to the composite.
        texture->setImage( layerNum, image );

        // track the maximum texture size
        // TODO: currently pointless since we are resizing all layers to 256 anyway...
        if ( image->s() > texWidth )
            texWidth = image->s();

        if ( image->t() > texHeight )
            texHeight = image->t();
            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        const GeoExtent& layerExtent = preparedImage.getExtent();

        float xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        float yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();
        float xscale  = tileExtent.width() / layerExtent.width();
        float yscale  = tileExtent.height() / layerExtent.height();

        texInfoArray->setElement( u++, xoffset );
        texInfoArray->setElement( u++, yoffset );
        texInfoArray->setElement( u++, xscale );
        texInfoArray->setElement( u++, yscale );
    }

    texture->setTextureSize( texWidth, texHeight, layerImages.size() );

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
    //texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    // (note: do NOT call setTextureAttributeAndModes -- opengl error since TEXTURE_2D_ARRAY is not a valid mode)
    //TODO: un-hard-code the texture unit, perhaps
    stateSet->setTextureAttribute( 0, texture, osg::StateAttribute::ON ); 
    stateSet->addUniform( texInfoArray );
    stateSet->addUniform( new osg::Uniform("tex0", 0) );

    return stateSet;
}
#endif

void
TextureCompositorTexArray::updateMasterStateSet( osg::StateSet* stateSet, const TextureLayout& layout ) const
{
    VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(osg::StateAttribute::PROGRAM) );
    vp->setShader( "osgearth_frag_texture", s_createTextureFragShaderFunction(layout) );
}

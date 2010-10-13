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
#include <osg/Texture2DArray>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorTexArray] "

//------------------------------------------------------------------------

static char s_source_vertMain[] =
    
    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

#if 0
static char s_source_fragMain[] =

    "uniform float region[256]; \n"
    "uniform int   osgearth_region_count; \n"
    "uniform sampler2DArray tex0; \n"

    "uniform float osgearth_imagelayer_opacity[128]; \n"

    "void main(void) \n"
    "{ \n"
    "    vec3 color = vec3(1,1,1); \n"
    "    for(int i=0; i<osgearth_region_count; i++) \n"
    "    { \n"
    "        int j = 8*i; \n"
    "        float tx   = region[j];   \n"
    "        float ty   = region[j+1]; \n"
    "        float tw   = region[j+2]; \n"
    "        float th   = region[j+3]; \n"
    "        float xoff = region[j+4]; \n"
    "        float yoff = region[j+5]; \n"
    "        float xsca = region[j+6]; \n"
    "        float ysca = region[j+7]; \n"

    "        float opac = osgearth_imagelayer_opacity[i]; \n"

    "        float u = tx + ( xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
    "        float v = ty + ( yoff + ysca * gl_TexCoord[0].t ) * th; \n"

    "        vec4 texel = texture2DArray( tex0, vec3(u,v,i) ); \n"
    "        color = mix(color, texel.rgb, texel.a * opac); \n"
    "    } \n"
    "    gl_FragColor = vec4(color, 1); \n"
    "} \n";
#endif

#if 0
static std::string
s_createFragShader( int numImageLayers )
{
    std::stringstream buf;

    buf << "#version 130 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n"

        << "uniform float region[" << numImageLayers*8 << "]; \n"
        << "uniform sampler2DArray tex0; \n"
        << "uniform float osgearth_imagelayer_opacity[" << numImageLayers << "]; \n"

        << "void main(void) \n"
        << "{ \n"
        <<     "vec3 color = vec3(1,1,1); \n"
        <<     "float u, v; \n"
        <<     "vec4 texel; \n"
        <<     "for(int i=0; i<"<< numImageLayers <<"; i++) { \n"
        <<         "int j = 8*i; \n"
        <<         "float tx = region[j]; \n"
        <<         "float ty = region[j+1]; \n"
        <<         "float tw = region[j+2]; \n"
        <<         "float th = region[j+3]; \n"
        <<         "float xoff = region[j+4]; \n"
        <<         "float yoff = region[j+5]; \n"
        <<         "float xsca = region[j+6]; \n"
        <<         "float ysca = region[j+7]; \n"
        <<         "float opac = osgearth_imagelayer_opacity[i]; \n"
        <<         "float u = tx + (xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
        <<         "float v = ty + (yoff + ysca * gl_TexCoord[0].t ) * th; \n"
        <<         "vec4 texel = texture2DArray( tex0, vec3(u,v,i) ); \n"
        <<         "color = mix(color, texel.rgb, texel.a * opac); \n"
        <<    "} \n"
        <<    "gl_FragColor = vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    OE_NOTICE << std::endl << str;
    return str;
}
#endif

static std::string
s_createFragShader( int numImageLayers )
{
    std::stringstream buf;

    buf << "#version 130 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n"

        << "uniform sampler2DArray tex0; \n"
        << "uniform float[] region; \n"
        << "uniform float[] osgearth_imagelayer_opacity; \n"

        << "void main(void) \n"
        << "{ \n"
        <<     "vec3 color = vec3(1,1,1); \n"
        <<     "float u, v; \n"
        <<     "vec4 texel; \n";

    for(int i=0; i<numImageLayers; ++i)
    {
        int j = i*4;
        buf << "u = region["<< j <<"] + (region["<< j+2 <<"] * gl_TexCoord[0].s); \n"
            << "v = region["<< j+1 <<"] + (region["<< j+3 <<"] * gl_TexCoord[0].t); \n"
            << "texel = texture2DArray( tex0, vec3(u,v,"<< i <<") ); \n"
            << "color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i <<"]); \n";
    }

    buf <<     "gl_FragColor = vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    OE_NOTICE << std::endl << str;
    return str;
}

//------------------------------------------------------------------------

GeoImage
TextureCompositorTexArray::prepareLayerUpdate( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
{
    osg::ref_ptr<osg::Image> image = layerImage.getImage();

    // Because all tex2darray layers must be identical in format, let's use RGBA.
    if ( image->getPixelFormat() != GL_RGBA )
        image = ImageUtils::convertToRGBA( image.get() );

    // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
    if ( image->s() != 256 || image->t() != 256 )
        image = ImageUtils::resizeImage( image.get(), 256, 256 );

    return GeoImage( image.get(), layerImage.getExtent() );
}

void
TextureCompositorTexArray::applyLayerUpdate(osg::StateSet* stateSet,
                                            int layerNum,
                                            const GeoImage& preparedImage,
                                            const GeoExtent& tileExtent) const
{
    // here we are going to assume everything's kosher - i.e. that the state set came
    // from createStateSet() in this class, and that the prepared image came from prepareImage()
    // in this same class.

    //TODO: un-hard-code the texture unit, perhaps
    osg::Texture2DArray* texture = static_cast<osg::Texture2DArray*>(
        stateSet->getTextureAttribute( 0, osg::StateAttribute::TEXTURE ) );

    // assign the new image at the proper position in the texture array:
    texture->setImage( layerNum, preparedImage.getImage() );
    
    // update the region uniform to reflect the geo extent of the image:
    const GeoExtent& imageExtent = preparedImage.getExtent();
    float xoffset = (tileExtent.xMin() - imageExtent.xMin()) / imageExtent.width();
    float yoffset = (tileExtent.yMin() - imageExtent.yMin()) / imageExtent.height();
    float xscale  = tileExtent.width() / imageExtent.width();
    float yscale  = tileExtent.height() / imageExtent.height();

    osg::Uniform* texInfoArray = stateSet->getUniform( "region" );
    if ( texInfoArray )
    {
        int layerOffset = layerNum * 4;
        texInfoArray->setElement( layerOffset,     xoffset );
        texInfoArray->setElement( layerOffset + 1, yoffset );
        texInfoArray->setElement( layerOffset + 2, xscale );
        texInfoArray->setElement( layerOffset + 3, yscale );
        texInfoArray->dirty();
    }
}

osg::StateSet*
TextureCompositorTexArray::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();

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

void
TextureCompositorTexArray::updateGlobalStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_source_vertMain ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_createFragShader(numImageLayers) ) );
    stateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
}

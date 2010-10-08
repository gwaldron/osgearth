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

// Records the information about a layer's texture space
struct LayerTexRegion
{
    LayerTexRegion() :
        _px(0), _py(0),
        _pw(256), _ph(256),
        _tx(0.0f), _ty(0.0f),
        _tw(1.0f), _th(1.0f),
        _xoffset(0.0f), _yoffset(0.0f),
        _xscale(1.0f), _yscale(1.0f)
    {
        //nop
    }

    // pixel coordinates of layer in the composite image:
    int _px, _py, _pw, _ph;

    // texture coordinates of layer in the composite image:
    float _tx, _ty, _tw, _th;

    // texture scale and offset for this region:
    float _xoffset, _yoffset, _xscale, _yscale;
};
typedef std::vector<LayerTexRegion> LayerTexRegionList;

//------------------------------------------------------------------------

static char s_source_vertMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

static char s_source_fragMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "uniform float osgearth_region[256]; \n"
    "uniform int   osgearth_region_count; \n"
    "uniform sampler2DArray tex0; \n"

    "uniform float osgearth_imagelayer_opacity[128]; \n"

    "void main(void) \n"
    "{ \n"
    "    vec3 color = vec3(1,1,1); \n"
    "    for(int i=0; i<osgearth_region_count; i++) \n"
    "    { \n"
    "        int j = 8*i; \n"
    "        float tx   = osgearth_region[j];   \n"
    "        float ty   = osgearth_region[j+1]; \n"
    "        float tw   = osgearth_region[j+2]; \n"
    "        float th   = osgearth_region[j+3]; \n"
    "        float xoff = osgearth_region[j+4]; \n"
    "        float yoff = osgearth_region[j+5]; \n"
    "        float xsca = osgearth_region[j+6]; \n"
    "        float ysca = osgearth_region[j+7]; \n"

    "        float opac = osgearth_imagelayer_opacity[i]; \n"

    "        float u = tx + ( xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
    "        float v = ty + ( yoff + ysca * gl_TexCoord[0].t ) * th; \n"

    "        vec4 texel = texture2DArray( tex0, vec3(u,v,i) ); \n"
    "        color = mix(color, texel.rgb, texel.a * opac); \n"
    "    } \n"
    "    gl_FragColor = vec4(color, 1); \n"
    "} \n";

//------------------------------------------------------------------------

GeoImage
TextureCompositorTexArray::prepareLayerUpdate( const GeoImage& layerImage, const GeoExtent& tileExtent ) const
{
    osg::ref_ptr<osg::Image> image = layerImage.getImage();

    // Because all tex2darray layers must be identical in format, let's use RGBA.
    if ( image->getPixelFormat() != GL_RGBA )
        image = ImageUtils::convertToRGBA( image );

    // TODO: revisit. For now let's just settle on 256 (again, all layers must be the same size)
    if ( image->s() != 256 || image->t() != 256 )
        image = ImageUtils::resizeImage( image, 256, 256 );

    return GeoImage( image.get(), layerImage.getExtent() );
}

void
TextureCompositorTexArray::applyLayerUpdate(osg::StateSet* stateSet,
                                            int layerNum,
                                            const GeoImage& preparedImage,
                                            const GeoExtent& tileExtent) const
{
    // here we are going to assume everything's kosher based - i.e. that the state set came
    // from createStateSet() in this class, and that the prepared image came from prepareImage()
    // in this same class.

    //TODO: un-hard-code the texture unit, perhaps
    osg::Texture2DArray* texture = static_cast<osg::Texture2DArray*>(
        stateSet->getTextureAttribute( 0, osg::StateAttribute::TEXTURE ) );

    // assign the new image at the proper position in the texture array:
    texture->setImage( layerNum, preparedImage.getImage() );
    
    // update the region uniform to reflect the geo extent of the image:
    const GeoExtent& layerExtent = preparedImage.getExtent();
    float xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
    float yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();
    float xscale  = tileExtent.width() / layerExtent.width();
    float yscale  = tileExtent.height() / layerExtent.height();

    osg::Uniform* texInfoArray = stateSet->getUniform( "osgearth_region" );
    int layerOffset = (layerNum * 8) + 4; // skip the pixel data, it won't change..
    texInfoArray->setElement( layerOffset++, xoffset );
    texInfoArray->setElement( layerOffset++, yoffset );
    texInfoArray->setElement( layerOffset++, xscale );
    texInfoArray->setElement( layerOffset++, yscale );
}

osg::StateSet*
TextureCompositorTexArray::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();

    // Composite all the image layer images into a single composite image.
    //
    // NOTE!
    // This should work if images are different sizes, BUT it will NOT work if they use
    // different locators. In other words, this will only work if the texture coordinate
    // pair (u,v) is the SAME across all image layers for a given vertex. That's because
    // GLSL will only support one tex-coord pair per texture unit, and we are doing the
    // compositing so we only need to use one texture unit.

    osg::Texture2DArray* texture = new osg::Texture2DArray();
    LayerTexRegionList regions;

    int texWidth = 0, texHeight = 0;

    // find each image layer and create a region entry for it
    texture->setTextureDepth( layerImages.size() );
    texture->setInternalFormat( GL_RGBA );

    int layerNum = 0;
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage& geoImage = *i;
        GeoImage preparedImage = prepareLayerUpdate( geoImage, tileExtent );
        osg::Image* image = preparedImage.getImage();

        // add the layer image to the composite.
        texture->setImage( layerNum,image );

        // TODO: optimize this away
        LayerTexRegion region;
        region._px = 0;
        region._py = 0;
        region._pw = image->s();
        region._ph = image->t();

        // track the maximum texture size
        // TODO: currently pointless since we are resizing all layers to 256 anyway...
        if ( image->s() > texWidth )
            texWidth = image->s();

        if ( image->t() > texHeight )
            texHeight = image->t();
            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        const GeoExtent& layerExtent = geoImage.getExtent();

        region._xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        region._yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();

        region._xscale = tileExtent.width() / layerExtent.width();
        region._yscale = tileExtent.height() / layerExtent.height();
            
        regions.push_back( region );
    }

    texture->setTextureSize( texWidth, texHeight, layerImages.size() );

    // configure the mipmapping 
    texture->setMaxAnisotropy(16.0f);
    texture->setResizeNonPowerOfTwoHint(false);
    bool powerOfTwo = texWidth > 0 && (texWidth & (texWidth - 1)) && texHeight > 0 && (texHeight & (texHeight - 1));
    if ( powerOfTwo )
        texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    else
        texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );

    // configure the wrapping
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    // build the uniforms.
    //    
    // The uniform array contains 8 floats for each region:
    //   tx, ty : origin texture coordinates in the composite-image space
    //   tw, th : width and height in composite-image space
    //   xoff, yoff : x- and y- offsets within texture space
    //   xsca, ysca : x- and y- scale factors within texture space

    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_region", regions.size() * 8 );
    int p=0;
    for( unsigned int i=0; i<regions.size(); ++i )
    {
        LayerTexRegion& region = regions[i];

        // next calculate the texture space extents and store those in uniforms.
        // (GW: there is no actual reason to store these in the region structure)
        region._tx = (float)region._px/(float)texWidth;
        region._ty = (float)region._py/(float)texHeight;
        region._tw = (float)region._pw/(float)texWidth;
        region._th = (float)region._ph/(float)texHeight;

        texInfoArray->setElement( p++, region._tx );
        texInfoArray->setElement( p++, region._ty );
        texInfoArray->setElement( p++, region._tw );
        texInfoArray->setElement( p++, region._th );
        texInfoArray->setElement( p++, region._xoffset );
        texInfoArray->setElement( p++, region._yoffset );
        texInfoArray->setElement( p++, region._xscale );
        texInfoArray->setElement( p++, region._yscale );

        //OE_NOTICE << LC
        //    << "Region " << i << ": size=(" << region._pw << ", " << region._ph << ")" << std::endl;
    }

    // (note: do NOT call setTextureAttributeAndModes -- opengl error since TEXTURE_2D_ARRAY is not a valid mode)
    //TODO: un-hard-code the texture unit, perhaps
    stateSet->setTextureAttribute( 0, texture, osg::StateAttribute::ON ); 
    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)regions.size() );

    return stateSet;
}

osg::Program*
TextureCompositorTexArray::createProgram() const
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_source_vertMain ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_source_fragMain ) );
    return program;
}

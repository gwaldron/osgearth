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

#include <osgEarth/TextureCompositor>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture2DArray>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <vector>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TextureCompositor] "

//------------------------------------------------------------------------

// supported compositing techniques.
enum Technique
{
    TECH_UNSET =0,
    TECH_TEXTURE_2D_ARRAY,
    TECH_TEXTURE_3D,
    TECH_TEXTURE_2D_ATLAS,
    TECH_MULTI_TEXTURE,
    TECH_SINGLE_TEXTURE
};

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


static void
configureTexture( osg::Texture* texture, int w, int h )
{  
    texture->setMaxAnisotropy(16.0f);
    texture->setResizeNonPowerOfTwoHint(false);

    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );

    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    bool mipMapping = !(texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::LINEAR || texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::NEAREST);
    bool s_NotPowerOfTwo = w==0 || (w & (w - 1));
    bool t_NotPowerOfTwo = h==0 || (h & (h - 1));

    if (mipMapping && (s_NotPowerOfTwo || t_NotPowerOfTwo))
    {
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    }
}

//------------------------------------------------------------------------

static char source_vertMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

static char source_array_fragMain[] =

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

static osg::StateSet*
createTexture2DArray( const GeoImageList& layerImages, const GeoExtent& tileExtent )
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

    osg::Texture2DArray* composite = new osg::Texture2DArray();
    LayerTexRegionList regions;

    int texWidth = 0, texHeight = 0;

    // find each image layer and create a region entry for it
    //GeoLocator* masterLocator = 0L;
    //unsigned int numColorLayers = _terrainTile->getNumColorLayers();
    composite->setTextureDepth( layerImages.size() ); //numColorLayers );
    composite->setInternalFormat( GL_RGBA );

    int layerNum = 0;
    for( GeoImageList::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage* geoImage = i->get();
        osg::ref_ptr<osg::Image> image = geoImage->getImage();

        // Because all tex2darray layers must be identical in format:
        if ( image->getPixelFormat() != GL_RGBA )
            image = ImageUtils::convertToRGBA( image );

        // TODO: reconsider.. perhaps grow the tex to the max layer size instead?
        if ( image->s() != 256 || image->t() != 256 )
            image = ImageUtils::resizeImage( image, 256, 256 );

        // add the layer image to the composite.
        composite->setImage( layerNum, image.get() );

        // TODO: optimize this away
        LayerTexRegion region;
        region._px = 0;
        region._py = 0;
        region._pw = image->s();
        region._ph = image->t();

        // track the maximum texture size
        if ( image->s() > texWidth )
            texWidth = image->s();

        if ( image->t() > texHeight )
            texHeight = image->t();
            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        const GeoExtent& layerExtent = geoImage->getExtent();

        region._xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        region._yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();

        region._xscale = tileExtent.width() / layerExtent.width();
        region._yscale = tileExtent.height() / layerExtent.height();
            
        regions.push_back( region );
    }

    composite->setTextureSize( texWidth, texHeight, layerImages.size() );

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

    configureTexture( composite, texWidth, texHeight );

    // (note: do NOT call setTextureAttributeAndModes -- opengl error since TEXTURE_2D_ARRAY is not a valid mode)
    stateSet->setTextureAttribute( 0, composite, osg::StateAttribute::ON ); //TODO: un-hard-code the texture unit, perhaps
    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)regions.size() );

    return stateSet;
}

//------------------------------------------------------------------------

static osg::StateSet*
createTexture3D( const GeoImageList& layerImages, const GeoExtent& tileExtent )
{
    OE_WARN << LC << "Texture 3D technique not yet implemented!" << std::endl;
    return new osg::StateSet();
}

//------------------------------------------------------------------------

static char source_atlas_fragMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "uniform float osgearth_region[256]; \n"
    "uniform int   osgearth_region_count; \n"
    "uniform sampler2D tex0; \n"

    "uniform float osgearth_imagelayer_opacity[128]; \n"

    "void main(void) \n"
    "{ \n"
    "    vec3 color = vec3(1,1,1); \n"
    "    for(int i=0; i<osgearth_region_count; i++) \n"
    "    { \n"
    "        int r = 8*i; \n"
    "        float tx   = osgearth_region[r];   \n"
    "        float ty   = osgearth_region[r+1]; \n"
    "        float tw   = osgearth_region[r+2]; \n"
    "        float th   = osgearth_region[r+3]; \n"
    "        float xoff = osgearth_region[r+4]; \n"
    "        float yoff = osgearth_region[r+5]; \n"
    "        float xsca = osgearth_region[r+6]; \n"
    "        float ysca = osgearth_region[r+7]; \n"

    "        float opac = osgearth_imagelayer_opacity[i]; \n"

    "        float u = tx + ( xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
    "        float v = ty + ( yoff + ysca * gl_TexCoord[0].t ) * th; \n"

    "        vec4 texel = texture2D( tex0, vec2(u,v) ); \n"
    "        color = mix(color, texel.rgb, texel.a * opac); \n"
    "    } \n"
    "    gl_FragColor = vec4(color, 1); \n"
    "} \n";

static osg::StateSet*
createTextureAtlas( const GeoImageList& layerImages, const GeoExtent& tileExtent )
{
    // Composite all the image layer images into a single composite image.
    //
    // NOTE!
    // This should work if images are different sizes, BUT it will NOT work if they use
    // different locators. In other words, this will only work if the texture coordinate
    // pair (u,v) is the SAME across all image layers for a given vertex. That's because
    // GLSL will only support one tex-coord pair per texture unit, and we are doing the
    // compositing so we only need to use one texture unit.

    int cx=0, cy=0;
    int maxRowHeight=0;
    int maxLegalWidth=1024, maxLegalHeight=1024; // hard coded for the moment..just testing
    int totalWidth=0, totalHeight=0;

    LayerTexRegionList regions;

    int layerNum = 0;
    for( GeoImageList::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage* geoImage = i->get();
        osg::ref_ptr<osg::Image> image = geoImage->getImage();

        LayerTexRegion region;

        if ( cx + image->s() <= maxLegalWidth )
        {
            // append this tile to the current row
            region._px = cx;
            region._py = cy;
            region._pw = image->s(); 
            region._ph = image->t();
            if ( maxRowHeight < region._ph )
                maxRowHeight = region._ph;
        }
        else
        {
            // ran out of width; start a new row
            cx = 0;
            cy += maxRowHeight;
            maxRowHeight = 0.0;
            region._px = cx;
            region._py = cy;
            region._pw = image->s();
            region._ph = image->t();
        }
        cx += region._pw;

        const GeoExtent& layerExtent = geoImage->getExtent();

        region._xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        region._yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();

        region._xscale = tileExtent.width() / layerExtent.width();
        region._yscale = tileExtent.height() / layerExtent.height();
            
        regions.push_back( region );

        totalWidth = osg::maximum( totalWidth, cx );
        totalHeight = osg::maximum( totalHeight, cy + maxRowHeight );
    }

    // now, calculate the size of the composite image and allocate it.
    // TODO: account for different image pixel formats by converting everything to RGBA
    osg::Image* out_image = new osg::Image();
    out_image->allocateImage( totalWidth, totalHeight, 1, GL_RGBA, GL_UNSIGNED_BYTE );

    // build the uniforms.
    osg::StateSet* stateSet = new osg::StateSet();
    
    // The uniform array contains 8 floats for each region:
    //   tx, ty : origin texture coordinates in the composite-image space
    //   tw, th : width and height in composite-image space
    //   xoff, yoff : x- and y- offsets within texture space
    //   xsca, ysca : x- and y- scale factors within texture space
    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_region", regions.size() * 8 );
    int p=0, r=0;
    for( GeoImageList::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++r )
    {
        LayerTexRegion& region = regions[r];

        // copy the image into the composite:
        ImageUtils::copyAsSubImage( i->get()->getImage(), out_image, region._px, region._py );

        // next calculate the texture space extents and store those in uniforms.
        // (GW: there is no actual reason to store these in the region structure)
        region._tx = (float)region._px/(float)totalWidth;
        region._ty = (float)region._py/(float)totalHeight;
        region._tw = (float)region._pw/(float)totalWidth;
        region._th = (float)region._ph/(float)totalHeight;

        texInfoArray->setElement( p++, region._tx );
        texInfoArray->setElement( p++, region._ty );
        texInfoArray->setElement( p++, region._tw );
        texInfoArray->setElement( p++, region._th );
        texInfoArray->setElement( p++, region._xoffset );
        texInfoArray->setElement( p++, region._yoffset );
        texInfoArray->setElement( p++, region._xscale );
        texInfoArray->setElement( p++, region._yscale );
    }

    osg::Texture* tex = new osg::Texture2D( out_image );
    configureTexture( tex, out_image->s(), out_image->t() );

    stateSet->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );
    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)regions.size() );

    return stateSet;
}

//------------------------------------------------------------------------

static osg::StateSet*
createMultiTexture( const GeoImageList& layerImages, const GeoExtent& outputExtent )
{
    OE_WARN << LC << "Multi-texture technique not yet implemented!" << std::endl;
    return new osg::StateSet();
}

//------------------------------------------------------------------------

static osg::StateSet*
createSingleTexture( const GeoImageList& layerImages, const GeoExtent& outputExtent )
{
    OE_WARN << LC << "Single-texture technique not yet implemented!" << std::endl;
    return new osg::StateSet();
}

//------------------------------------------------------------------------

TextureCompositor::TextureCompositor() :
_tech( TECH_UNSET )
{
    //nop
}

osg::StateSet*
TextureCompositor::createStateSet( const GeoImageList& layerImages, const GeoExtent& outputExtent ) const
{
    // first time through, poll the system capabilities to figure out
    // which technique to use.
    if ( _tech == TECH_UNSET )
        const_cast<TextureCompositor*>(this)->init();

    switch( _tech )
    {
    case TECH_TEXTURE_2D_ARRAY:
        return createTexture2DArray( layerImages, outputExtent );

    case TECH_TEXTURE_3D:
        return createTexture3D( layerImages, outputExtent );

    case TECH_TEXTURE_2D_ATLAS:
        return createTextureAtlas( layerImages, outputExtent );

    case TECH_MULTI_TEXTURE:
        return createMultiTexture( layerImages, outputExtent );

    default:
    case TECH_SINGLE_TEXTURE:
        return createSingleTexture( layerImages, outputExtent );
    }
}

osg::Program*
TextureCompositor::getProgram() const
{
    if ( _tech == TECH_UNSET )
        const_cast<TextureCompositor*>(this)->init();

    return _program.get();
}

void
TextureCompositor::init()
{        
    ScopedLock<Mutex> initLock( _initMutex );
    if ( _tech != TECH_UNSET ) // double-check pattern
    {
        return; // already initialized
    }

    osg::Shader* vertShader =0L;
    osg::Shader* fragShader =0L;

    const Capabilities& caps = Registry::instance()->getCapabilities();

    if ( caps.supportsTextureArrays() )
    {
        _tech = TECH_TEXTURE_2D_ARRAY;
        vertShader = new osg::Shader( osg::Shader::VERTEX, std::string( source_vertMain ) );
        fragShader = new osg::Shader( osg::Shader::FRAGMENT, std::string( source_array_fragMain ) );
        OE_INFO << LC << "technique = texture2darray" << std::endl;
    }
    else if ( caps.supportsTexture3D() )
    {
        _tech = TECH_TEXTURE_3D;
        OE_INFO << LC << "technique = texture3d" << std::endl;
    }
    else if ( caps.supportsGLSL() )
    {
        _tech = TECH_TEXTURE_2D_ATLAS;
        vertShader = new osg::Shader( osg::Shader::VERTEX, std::string( source_vertMain ) );
        fragShader = new osg::Shader( osg::Shader::FRAGMENT, std::string( source_atlas_fragMain ) );
        OE_INFO << LC << "technique = atlas" << std::endl;
    }
    else if ( caps.supportsMultiTexture() )
    {
        _tech = TECH_MULTI_TEXTURE;
        OE_INFO << LC << "technique = multitexture" << std::endl;
    }
    else
    {
        _tech = TECH_SINGLE_TEXTURE;
        OE_INFO << LC << "technique = single texture" << std::endl;
    }


    if ( vertShader && fragShader )
    {
        _program = new osg::Program();
        _program->addShader( vertShader );
        _program->addShader( fragShader );
    }
}

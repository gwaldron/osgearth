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

#include <osgEarth/TextureCompositorAtlas>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture2D>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorAtlas] "

//------------------------------------------------------------------------

namespace
{
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
}

//------------------------------------------------------------------------

static char s_source_vertMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

#if 0
static char s_source_fragMain[] =

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
#endif

static std::string
s_createFragShader( int numImageLayers )
{
    std::stringstream buf;

    buf << "uniform float osgearth_region[" << numImageLayers*8 << "]; \n"
        << "uniform sampler2D tex0; \n"
        << "uniform float osgearth_imagelayer_opacity[" << numImageLayers << "]; \n"

        << "void main(void) \n"
        << "{ \n"
        <<     "vec3 color = vec3(1,1,1); \n"
        <<     "float u, v; \n"
        <<     "vec4 texel; \n";

    for(int i=0; i<numImageLayers; ++i)
    {
        int j = i*8;
        buf << "u = osgearth_region["<< j <<"] + (osgearth_region["<< j+4 <<"] + osgearth_region["<< j+6 <<"] * gl_TexCoord[0].s) * osgearth_region["<< j+2 << "]; \n"
            << "v = osgearth_region["<< j+1 <<"] + (osgearth_region["<< j+5 <<"] + osgearth_region["<< j+7 <<"] * gl_TexCoord[0].t) * osgearth_region["<< j+3 << "]; \n"
            << "texel = texture2D( tex0, vec2(u,v) ); \n"
            << "color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i << "]); \n";
    }

    buf <<     "gl_FragColor = vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    return str;
}


//------------------------------------------------------------------------

osg::StateSet*
TextureCompositorAtlas::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
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
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage& geoImage = *i;
        osg::ref_ptr<osg::Image> image = geoImage.getImage();

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

        const GeoExtent& layerExtent = geoImage.getExtent();

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
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++r )
    {
        LayerTexRegion& region = regions[r];

        // copy the image into the composite:
        ImageUtils::copyAsSubImage( i->getImage(), out_image, region._px, region._py );

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

    osg::Texture* texture = new osg::Texture2D( out_image );
    
    // configure the mipmapping 
    texture->setMaxAnisotropy(16.0f);
    texture->setResizeNonPowerOfTwoHint(false);
    bool powerOfTwo = totalWidth > 0 && (totalWidth & (totalWidth - 1)) && totalHeight > 0 && (totalHeight & (totalHeight - 1));
    if ( powerOfTwo )
        texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    else
        texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );

    // configure the wrapping
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    // build the stateset.
    stateSet->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );
    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)regions.size() );

    return stateSet;
}

void
TextureCompositorAtlas::updateGlobalStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_source_vertMain ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_createFragShader(numImageLayers) ) );
    stateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
}

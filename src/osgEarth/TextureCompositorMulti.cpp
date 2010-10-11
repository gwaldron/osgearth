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
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorMultiTexture] "

//------------------------------------------------------------------------

static std::string
s_createVertShader( int numImageLayers )
{
    std::stringstream buf;

    buf << "void main(void) \n"
        << "{ \n";

    for( int i=0; i<numImageLayers; ++i )
        buf << "gl_TexCoord[" << i << "] = gl_MultiTexCoord" << i << "; \n";

    buf <<     "gl_Position = ftransform(); \n"
        << "} \n";

    std::string str = buf.str();
    return str;
}

//------------------------------------------------------------------------

static std::string
s_createFragShader( int numImageLayers ) 
{
    std::stringstream buf;

    buf << "uniform int osgearth_region_count; \n"
        << "uniform float osgearth_imagelayer_opacity[" << numImageLayers << "]; \n"
        << "uniform sampler2D ";

    for( int i=0; i<numImageLayers; ++i )
        buf << "tex" << i << ( i+1 < numImageLayers? "," : ";") << " \n";

    buf << "void main(void) \n"
        << "{ \n"
        <<     "vec3 color = vec3(1,1,1); \n"
        <<     "vec4 texel; \n";

    for( int i=0; i<numImageLayers; ++i )
    {
        buf << "texel = texture2D(tex" << i << ", gl_TexCoord[" << i << "].st); \n"
            << "color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity[" << i << "]); \n";
    }

    buf <<     "gl_FragColor = vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    return str;
}

//------------------------------------------------------------------------

TextureCompositorMultiTexture::TextureCompositorMultiTexture( bool useGPU ) :
_useGPU( useGPU )
{
}

osg::StateSet*
TextureCompositorMultiTexture::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();

    int texUnit =0;
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++texUnit )
    {
        osg::Texture2D* texture = new osg::Texture2D( i->getImage() );
        int texWidth = i->getImage()->s();
        int texHeight = i->getImage()->t();

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
    }

    return stateSet;
}

void 
TextureCompositorMultiTexture::updateGlobalStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    if ( _useGPU )
    {
        osg::Program* program = new osg::Program();
        program->addShader( new osg::Shader( osg::Shader::VERTEX, s_createVertShader(numImageLayers) ) );
        program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_createFragShader(numImageLayers) ) );
        stateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
    }

    else
    {
        // FFP multitexturing requires that we set up a series of TexCombine attributes:

        if (numImageLayers == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateSet->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (numImageLayers >= 2)
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
            for (int unit = 1; unit < numImageLayers-1; ++unit)
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
                stateSet->setTextureAttributeAndModes(numImageLayers-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
}

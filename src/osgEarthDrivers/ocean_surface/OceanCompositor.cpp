/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include "OceanCompositor"
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osg/Texture2D>
#include "OceanShaders"

using namespace osgEarth;

void
OceanCompositor::updateMasterStateSet(osg::StateSet*       stateSet, 
                                      const TextureLayout& layout ) const
{
    VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(VirtualProgram::SA_TYPE) );
    if ( !vp )
    {
        vp = new VirtualProgram();
        vp->setName("osgEarth OceanCompositor");
        stateSet->setAttributeAndModes( vp, 1 );
    }
    vp->installDefaultLightingShaders();
    vp->setShader( "osgearth_vert_setupColoring", new osg::Shader(osg::Shader::VERTEX, source_setupColoring) );
    vp->setShader( "osgearth_frag_applyColoring", new osg::Shader(osg::Shader::FRAGMENT, source_applyColoring ) );
}
    
namespace
{
    std::string makeSamplerName(int slot)
    {
        std::stringstream buf;
        buf << "ocean_tex" << slot;
        std::string str;
        str = buf.str();
        return str;
    }

    osg::Texture2D*
    s_getTexture( osg::StateSet* stateSet, UID layerUID, const TextureLayout& layout, osg::StateSet* parentStateSet)
    {
        int slot = layout.getSlot( layerUID, 0 );
        if ( slot < 0 )
            return 0L;

        osg::Texture2D* tex = static_cast<osg::Texture2D*>(
            stateSet->getTextureAttribute( slot, osg::StateAttribute::TEXTURE ) );

        if ( !tex )
        {
            tex = new osg::Texture2D();

            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );

            // configure the wrapping
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );

            stateSet->setTextureAttributeAndModes( slot, tex, osg::StateAttribute::ON );
            
            // install the slot attribute
            std::string name = makeSamplerName( slot );
            stateSet->getOrCreateUniform( name.c_str(), osg::Uniform::SAMPLER_2D )->set( slot );
        }
        return tex;
    }
}


void
OceanCompositor::applyLayerUpdate(osg::StateSet*       stateSet,
                                  UID                  layerUID,
                                  const GeoImage&      preparedImage,
                                  const TileKey&       tileKey,
                                  const TextureLayout& layout,
                                  osg::StateSet*       parentStateSet) const
{
    osg::Texture2D* tex = s_getTexture( stateSet, layerUID, layout, parentStateSet);
    if ( tex )
    {
        osg::Image* image = preparedImage.getImage();
        image->dirty(); // required for ensure the texture recognizes the image as new data
        tex->setImage( image );

        // set up proper mipmapping filters:
        if (ImageUtils::isPowerOfTwo( image ) && 
            !(!image->isMipmap() && ImageUtils::isCompressed(image)) )
        {
            if ( tex->getFilter(osg::Texture::MIN_FILTER) != osg::Texture::LINEAR_MIPMAP_LINEAR )
                tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
        }
        else if ( tex->getFilter(osg::Texture::MIN_FILTER) != osg::Texture::LINEAR )
        {
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
        }
    }
}

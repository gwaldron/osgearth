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
#include "OceanCompositor"
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osg/Texture2D>
#include "OceanShaders"

#define OCEAN_DATA "ocean_data"
#define OCEAN_TEX  "ocean_surface_tex"

using namespace osgEarth_ocean_surface;
using namespace osgEarth;
using namespace osgEarth::Drivers;


OceanCompositor::OceanCompositor(const OceanSurfaceOptions& options) :
_options( options )
{
    //nop
}

void
OceanCompositor::updateMasterStateSet(osg::StateSet*       stateSet, 
                                      const TextureLayout& layout ) const
{
    VirtualProgram* vp = VirtualProgram::getOrCreate( stateSet );
    vp->setName( "osgEarth.OceanCompositor" );

    // install a default lighting shader
    Registry::shaderFactory()->installLightingShaders( vp );

    // use the appropriate shader for the active technique:
    std::string vertSource = _options.maskLayer().isSet() ? source_vertMask : source_vertProxy;
    std::string fragSource = _options.maskLayer().isSet() ? source_fragMask : source_fragProxy;

    vp->setFunction( "oe_ocean_vertex",   vertSource, ShaderComp::LOCATION_VERTEX_VIEW );
    vp->setFunction( "oe_ocean_fragment", fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING );

    // install the slot attribute(s)
    stateSet->getOrCreateUniform( OCEAN_DATA, osg::Uniform::SAMPLER_2D )->set( 0 );
    stateSet->getOrCreateUniform( OCEAN_TEX,  osg::Uniform::SAMPLER_2D )->set( 1 );
}

namespace
{
    // probably don't need this
    std::string makeSamplerName(int slot)
    {
        if ( slot == 0 )
            return OCEAN_DATA;
        else 
            return OCEAN_TEX;
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

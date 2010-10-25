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
#include <osgEarth/TextureCompositorTexArray>
#include <osgEarth/TextureCompositorTex3D>
#include <osgEarth/TextureCompositorAtlas>
#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/TextureCompositorSoftware>
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

TextureCompositor::TextureCompositor( const TerrainOptions::CompositingTechnique& tech ) :
osg::Referenced( true ),
_tech( tech ),
_forceTech( false )
{
    // for debugging:
    if ( _tech == TerrainOptions::COMPOSITING_AUTO && ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) )
    {
        TerrainOptions::CompositingTechnique oldTech = _tech;
        std::string t( ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) );
        if      ( t == "TEXTURE_ARRAY" )    _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        //else if ( t == "TEXTURE_3D" )       _tech = TECH_TEXTURE_3D;
        //else if ( t == "TEXTURE_ATLAS" )    _tech = TerrainOptions::COMPOSITING_TEXTURE_ATLAS;
        else if ( t == "MULTITEXTURE_GPU" ) _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        else if ( t == "MULTITEXTURE_FFP" ) _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_FFP;
        else if ( t == "MULTIPASS" )        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        //else if ( t == "SOFTWARE" )         _tech = TECH_SOFTWARE;
        if ( oldTech != _tech )
            _forceTech = true;
    }

    init();
}

osg::StateSet*
TextureCompositor::createStateSet( const GeoImageVector& stack, const GeoExtent& tileExtent ) const
{
    return _impl ? _impl->createStateSet( stack, tileExtent ) : 0L;
}

bool
TextureCompositor::supportsLayerUpdate() const
{
    return _impl ? _impl->supportsLayerUpdate() : false;
}

GeoImage
TextureCompositor::prepareLayerUpdate( const GeoImage& image, const GeoExtent& tileExtent ) const
{
    return _impl ? _impl->prepareLayerUpdate( image, tileExtent ) : GeoImage::INVALID;
}

void
TextureCompositor::applyLayerUpdate(osg::StateSet* stateSet,
                                    int layerNum,
                                    const GeoImage& preparedImage,
                                    const GeoExtent& tileExtent ) const
{
    if ( _impl )
        _impl->applyLayerUpdate( stateSet, layerNum, preparedImage, tileExtent );
}

bool
TextureCompositor::requiresUnitTextureSpace() const
{
    return _impl->requiresUnitTextureSpace();
}

bool
TextureCompositor::usesShaderComposition() const
{
    return _impl->usesShaderComposition();
}

void
TextureCompositor::updateGlobalStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    _impl->updateGlobalStateSet( stateSet, numImageLayers );
}

void
TextureCompositor::init()
{        
    if ( _impl.valid() ) // double-check pattern
    {
        return; // already initialized
    }

    bool isAuto = _tech == TerrainOptions::COMPOSITING_AUTO;

    const Capabilities& caps = Registry::instance()->getCapabilities();

    if (_tech == TerrainOptions::COMPOSITING_TEXTURE_ARRAY || 
        (isAuto && caps.supportsGLSL(1.30f) && caps.supportsTextureArrays()) )
    {
        _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        _impl = new TextureCompositorTexArray();
        OE_INFO << LC << "Compositing technique = TEXTURE ARRAY" << std::endl;
    }

#if 0
    // check "forceTech" because it doesn't work yet:
    else if ( _forceTech && ( _tech == TerrainOptions::COMPOSITING_TEXTURE_3D || (isAuto && caps.supportsTexture3D()) ) )
    {
        _tech = TerrainOptions::COMPOSITING_TEXTURE_3D;
        _impl = new TextureCompositorTex3D();
        OE_INFO << LC << "Compositing technique = TEXTURE 3D" << std::endl;
    }

    // check "forceTech" because the tile boundaries show
    else if ( _forceTech && ( _tech == TerrainOptions::COMPOSITING_TEXTURE_ATLAS || (isAuto && caps.supportsGLSL()) ) )
    {
        _tech = TerrainOptions::COMPOSITING_TEXTURE_ATLAS;
        _impl = new TextureCompositorAtlas();
        OE_INFO << LC << "Compositing technique = TEXTURE ATLAS" << std::endl;
    }
#endif

    // commented out because it's NYI:
    else if (
        _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_GPU ||
        (isAuto && caps.supportsGLSL(1.20f) && caps.supportsMultiTexture()) ) 
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        _impl = new TextureCompositorMultiTexture( true );
        OE_INFO << LC << "Compositing technique = MULTITEXTURE/GPU" << std::endl;
    }

    // NOTE: uncomment this to "else if" when Software mode is ready
    else if ( _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_FFP || (isAuto && caps.supportsMultiTexture()) )
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_FFP;
        _impl = new TextureCompositorMultiTexture( false );
        OE_INFO << LC << "Compositing technique = MULTITEXTURE/FFP" << std::endl;
    }

    // Fallback of last resort. The Compositor does not actually support multipass.
    else
    {
        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        _impl = 0L;
        OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }
}

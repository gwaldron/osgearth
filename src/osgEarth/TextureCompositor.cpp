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

TextureCompositor::TextureCompositor( TextureCompositor::TechniqueType tech ) :
_tech( tech ),
_forceTech( false )
{
    // for debugging:
    if ( _tech == TECH_AUTO && ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) )
    {
        TechniqueType oldTech = _tech;
        std::string t( ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) );
        if      ( t == "TEXTURE_ARRAY" )    _tech = TECH_TEXTURE_ARRAY;
        else if ( t == "TEXTURE_3D" )       _tech = TECH_TEXTURE_3D;
        else if ( t == "TEXTURE_ATLAS" )    _tech = TECH_TEXTURE_ATLAS;
        else if ( t == "MULTITEXTURE_GPU" ) _tech = TECH_MULTITEXTURE_GPU;
        else if ( t == "MULTITEXTURE_FFP" ) _tech = TECH_MULTITEXTURE_FFP;
        else if ( t == "SOFTWARE" )         _tech = TECH_SOFTWARE;
        if ( oldTech != _tech )
            _forceTech = true;
    }
}

osg::StateSet*
TextureCompositor::createStateSet( const GeoImageVector& stack, const GeoExtent& tileExtent ) const
{
    // first time through, poll the system capabilities to figure out
    // which technique to use.
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    return _impl ? _impl->createStateSet( stack, tileExtent ) : 0L;
}

bool
TextureCompositor::supportsLayerUpdate() const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    return _impl ? _impl->supportsLayerUpdate() : false;
}

GeoImage
TextureCompositor::prepareLayerUpdate( const GeoImage& image, const GeoExtent& tileExtent ) const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    return _impl ? _impl->prepareLayerUpdate( image, tileExtent ) : GeoImage::INVALID;
}

void
TextureCompositor::applyLayerUpdate(osg::StateSet* stateSet,
                                    int layerNum,
                                    const GeoImage& preparedImage,
                                    const GeoExtent& tileExtent ) const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    if ( _impl )
        _impl->applyLayerUpdate( stateSet, layerNum, preparedImage, tileExtent );
}

osg::Program*
TextureCompositor::getProgram() const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    return _program.get();
}

bool
TextureCompositor::requiresUnitTextureSpace() const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    return _impl->requiresUnitTextureSpace();
}

void
TextureCompositor::updateGlobalStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    if ( !_impl.valid() )
        const_cast<TextureCompositor*>(this)->init();

    _impl->updateGlobalStateSet( stateSet, numImageLayers );
}

void
TextureCompositor::init()
{        
    ScopedLock<Mutex> initLock( _initMutex );
    if ( _impl.valid() ) // double-check pattern
    {
        return; // already initialized
    }

    bool isAuto = _tech == TECH_AUTO;

    const Capabilities& caps = Registry::instance()->getCapabilities();

    if ( _tech == TECH_TEXTURE_ARRAY || (isAuto && caps.supportsTextureArrays()) )
    {
        _tech = TECH_TEXTURE_ARRAY;
        _impl = new TextureCompositorTexArray();
        OE_INFO << LC << "technique = TEXTURE ARRAY" << std::endl;
    }

    // check "forceTech" because it doesn't work yet:
    else if ( _forceTech && ( _tech == TECH_TEXTURE_3D || (isAuto && caps.supportsTexture3D()) ) )
    {
        _tech = TECH_TEXTURE_3D;
        _impl = new TextureCompositorTex3D();
        OE_INFO << LC << "technique = TEXTURE 3D" << std::endl;
    }

    // check "forceTech" because the tile boundaries show
    else if ( _forceTech && ( _tech == TECH_TEXTURE_ATLAS || (isAuto && caps.supportsGLSL()) ) )
    {
        _tech = TECH_TEXTURE_ATLAS;
        _impl = new TextureCompositorAtlas();
        OE_INFO << LC << "technique = TEXTURE ATLAS" << std::endl;
    }

    // commented out because it's NYI:
    else if ( _forceTech && ( _tech == TECH_MULTITEXTURE_GPU || (isAuto && caps.supportsGLSL() && caps.supportsMultiTexture()) ) )
    {
        _tech = TECH_MULTITEXTURE_GPU;
        _impl = new TextureCompositorMultiTexture( true );
        OE_INFO << LC << "technique = MULTITEXTURE/GPU" << std::endl;
    }

    // NOTE: uncomment this to "else if" when Software mode is ready
    else // if ( _tech == TECH_MULTITEXTURE_FFP || (isAuto && caps.supportsMultiTexture()) )
    {
        _tech = TECH_MULTITEXTURE_FFP;
        _impl = new TextureCompositorMultiTexture( false );
        OE_INFO << LC << "technique = MULTITEXTURE/FFP" << std::endl;
    }

    // commented out because it's NYI:
    //else
    //{
    //    _tech = TECH_SOFTWARE;
    //    _impl = new TextureCompositorSoftware();
    //    OE_INFO << LC << "technique = software" << std::endl;
    //}

    if ( _impl.valid() )
        _program = _impl->createProgram();
}
